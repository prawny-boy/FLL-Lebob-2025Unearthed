#!/usr/bin/env python3
"""Deploy the teleop hub program and stream Xbox controller input (tank + aux)."""

import argparse
import asyncio
import signal
import time
from pathlib import Path

import evdev
from evdev import ecodes

from pybricksdev.ble import find_device
from pybricksdev.connections.pybricks import PybricksHubBLE

POLL_INTERVAL = 0.02
DEFAULT_DEADBAND = 0.05
DEFAULT_BLE_TIMEOUT = 10.0
DEFAULT_HUB_NAME = "FatSean"

# Xbox (from your evtest)
CODE_LY = ecodes.ABS_Y
CODE_RY = ecodes.ABS_RY
CODE_LT = ecodes.ABS_Z
CODE_RT = ecodes.ABS_RZ

BTN_LB = ecodes.BTN_TL
BTN_RB = ecodes.BTN_TR


def _clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def _normalize_trigger(value, absinfo):
    # your evtest: min 0, max 1023
    if absinfo is None:
        return 0.0
    span = absinfo.max - absinfo.min
    if not span:
        return 0.0
    return _clamp((value - absinfo.min) / span, 0.0, 1.0)


def _normalize_stick(value, absinfo):
    if absinfo is None:
        return 0.0
    span = max(abs(absinfo.min), abs(absinfo.max))
    if not span:
        return 0.0
    return _clamp(value / span, -1.0, 1.0)


def _select_devices(device_path=None):
    if device_path:
        return [evdev.InputDevice(device_path)]
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    xbox = [d for d in devices if "xbox" in d.name.lower()]
    if xbox:
        return xbox
    pads = [d for d in devices if "controller" in d.name.lower() or "gamepad" in d.name.lower()]
    if pads:
        return pads
    raise SystemExit("No controller-like input devices found.")


class InputState:
    def __init__(self):
        self.left_drive = 0.0
        self.right_drive = 0.0
        self.left_aux = 0.0
        self.right_aux = 0.0

        self._absinfo = {}
        self._raw = {}

    def update_absinfo(self, device):
        caps = device.capabilities(absinfo=True).get(ecodes.EV_ABS, [])
        for code, absinfo in caps:
            self._absinfo[code] = absinfo

    def handle_abs_event(self, code, value):
        self._raw[code] = value

        if code == CODE_LY:
            self.left_drive = -_normalize_stick(value, self._absinfo.get(code))
            return

        if code == CODE_RY:
            self.right_drive = -_normalize_stick(value, self._absinfo.get(code))
            return

        if code == CODE_LT:
            self._raw["lt"] = _normalize_trigger(value, self._absinfo.get(code))
            self._recompute_aux()
            return

        if code == CODE_RT:
            self._raw["rt"] = _normalize_trigger(value, self._absinfo.get(code))
            self._recompute_aux()
            return

    def handle_key_event(self, code, pressed):
        if code == BTN_LB:
            self._raw["lb"] = 1.0 if pressed else 0.0
            self._recompute_aux()
        elif code == BTN_RB:
            self._raw["rb"] = 1.0 if pressed else 0.0
            self._recompute_aux()

    def _recompute_aux(self):
        lt = float(self._raw.get("lt", 0.0))
        rt = float(self._raw.get("rt", 0.0))

        lb = float(self._raw.get("lb", 0.0))
        rb = float(self._raw.get("rb", 0.0))

        # Hold bumper to reverse direction.
        left_dir = -1.0 if lb > 0.5 else 1.0
        right_dir = -1.0 if rb > 0.5 else 1.0

        self.left_aux = _clamp(lt * left_dir, -1.0, 1.0)
        self.right_aux = _clamp(rt * right_dir, -1.0, 1.0)


class HubBridge:
    def __init__(self, hub):
        self.hub = hub

    async def send(self, ld, rd, la, ra):
        await self.hub.write_line(f"{ld:.3f},{rd:.3f},{la:.3f},{ra:.3f}")


def _read_evdev_loop(device, state, stop_event, debug=False):
    try:
        for event in device.read_loop():
            if stop_event.is_set():
                break

            if event.type == ecodes.EV_ABS:
                if debug:
                    print(f"EV_ABS code={event.code} value={event.value}")
                state.handle_abs_event(event.code, event.value)

            elif event.type == ecodes.EV_KEY:
                # FIX: treat 1 (press) and 2 (hold) as pressed
                pressed = event.value != 0
                if debug and event.code in (BTN_LB, BTN_RB):
                    print(f"EV_KEY code={event.code} value={event.value}")
                state.handle_key_event(event.code, pressed)
    except (OSError, ValueError):
        # Device closed or unavailable; exit the reader loop.
        return


async def _send_loop(state, bridge, deadband, stop_event, debug=False):
    last_send = 0.0
    last = None

    while not stop_event.is_set():
        now = time.monotonic()
        if now - last_send >= POLL_INTERVAL:
            ld = state.left_drive
            rd = state.right_drive
            la = state.left_aux
            ra = state.right_aux

            if abs(ld) < deadband:
                ld = 0.0
            if abs(rd) < deadband:
                rd = 0.0

            await bridge.send(ld, rd, la, ra)
            last_send = now

            if debug:
                cur = (round(ld, 3), round(rd, 3), round(la, 3), round(ra, 3))
                if last is None or cur != last:
                    print(f"TX ld={cur[0]:.3f} rd={cur[1]:.3f} la={cur[2]:.3f} ra={cur[3]:.3f}")
                    last = cur

        await asyncio.sleep(POLL_INTERVAL / 2)


async def main_async():
    parser = argparse.ArgumentParser(description="Deploy teleop and stream Xbox input")
    parser.add_argument("--hub", default=DEFAULT_HUB_NAME, help="Hub name or BLE address")
    parser.add_argument("--device", help="Input device path, e.g. /dev/input/event18")
    parser.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND)
    parser.add_argument("--ble-timeout", type=float, default=DEFAULT_BLE_TIMEOUT)
    parser.add_argument("--no-grab", action="store_true", help="Do not grab exclusive access")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    teleop_file = repo_root / "src" / "robot" / "teleop_hub.py"

    devices = _select_devices(args.device)

    if args.debug:
        for d in devices:
            print(f"Using input device: {d.path} ({d.name})")

    state = InputState()
    for d in devices:
        state.update_absinfo(d)

    stop_event = asyncio.Event()

    def _request_stop(*_args):
        stop_event.set()
        for d in devices:
            try:
                d.close()
            except OSError:
                pass

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _request_stop)
        except NotImplementedError:
            pass

    grabbed = []
    if not args.no_grab:
        for d in devices:
            try:
                d.grab()
                grabbed.append(d)
            except OSError:
                pass

    try:
        if args.debug:
            print(f"Searching for {args.hub}...")
        ble_device = await find_device(name=args.hub, timeout=args.ble_timeout)
        hub = PybricksHubBLE(ble_device)

        if args.debug:
            print("Connecting to hub...")
        await hub.connect()

        if args.debug:
            print("Deploying teleop_hub.py...")
        await hub.run(str(teleop_file), wait=False, print_output=args.debug)

        if args.debug:
            print("Teleop running. Streaming controller input...")

        bridge = HubBridge(hub)

        readers = [
            asyncio.create_task(
                asyncio.to_thread(_read_evdev_loop, d, state, stop_event, args.debug)
            )
            for d in devices
        ]
        sender = asyncio.create_task(
            _send_loop(state, bridge, args.deadband, stop_event, debug=args.debug)
        )

        await stop_event.wait()

        for r in readers:
            r.cancel()
        sender.cancel()

        await asyncio.gather(*readers, sender, return_exceptions=True)
        await bridge.send(0.0, 0.0, 0.0, 0.0)
        await hub.write_line("quit")
    finally:
        for d in grabbed:
            try:
                d.ungrab()
            except OSError:
                pass
        for d in devices:
            try:
                d.close()
            except OSError:
                pass


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
