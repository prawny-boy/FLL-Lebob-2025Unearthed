#!/usr/bin/env python3
"""Deploy the teleop hub program and stream Xbox controller input (tank + aux)."""

from argparse import ArgumentParser
import asyncio
import signal
import sys
import time
from pathlib import Path

try:
    import evdev
    from evdev import ecodes

    HAVE_EVDEV = True
except ImportError:
    evdev = None
    ecodes = None
    HAVE_EVDEV = False

try:
    from inputs import devices as inputs_devices
    from inputs import get_gamepad

    HAVE_INPUTS = True
except ImportError:
    inputs_devices = None
    get_gamepad = None
    HAVE_INPUTS = False

from pybricksdev.ble import find_device
from pybricksdev.connections.pybricks import PybricksHubBLE

POLL_INTERVAL = 0.02
DEFAULT_DEADBAND = 0.05
DEFAULT_BLE_TIMEOUT = 10.0
DEFAULT_HUB_NAME = "FatSean"

# Xbox (Linux evdev)
EVDEV_CODE_LY = ecodes.ABS_Y if HAVE_EVDEV else None
EVDEV_CODE_RY = ecodes.ABS_RY if HAVE_EVDEV else None
EVDEV_CODE_LT = ecodes.ABS_Z if HAVE_EVDEV else None
EVDEV_CODE_RT = ecodes.ABS_RZ if HAVE_EVDEV else None

EVDEV_BTN_LB = ecodes.BTN_TL if HAVE_EVDEV else None
EVDEV_BTN_RB = ecodes.BTN_TR if HAVE_EVDEV else None
EVDEV_BTN_A = ecodes.BTN_SOUTH if HAVE_EVDEV else None
EVDEV_BTN_B = ecodes.BTN_EAST if HAVE_EVDEV else None

# Xbox (python-inputs)
INPUTS_CODE_LY = "ABS_Y"
INPUTS_CODE_RY = "ABS_RY"
INPUTS_CODE_LT = "ABS_Z"
INPUTS_CODE_RT = "ABS_RZ"

INPUTS_BTN_LB = "BTN_TL"
INPUTS_BTN_RB = "BTN_TR"
INPUTS_BTN_A = "BTN_SOUTH"
INPUTS_BTN_B = "BTN_EAST"


class AbsInfo:
    def __init__(self, min_val, max_val):
        self.min = min_val
        self.max = max_val


INPUTS_ABSINFO = {
    INPUTS_CODE_LY: AbsInfo(-32768, 32767),
    INPUTS_CODE_RY: AbsInfo(-32768, 32767),
    INPUTS_CODE_LT: AbsInfo(0, 255),
    INPUTS_CODE_RT: AbsInfo(0, 255),
}


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


def _select_backend():
    if HAVE_EVDEV and sys.platform.startswith("linux"):
        return "evdev"
    if HAVE_INPUTS:
        return "inputs"
    if HAVE_EVDEV:
        return "evdev"
    raise SystemExit("No controller backend available. Install evdev (Linux) or inputs.")


def _select_devices(backend, device_path=None):
    if backend == "inputs":
        gamepads = inputs_devices.gamepads if HAVE_INPUTS else []
        if not gamepads:
            raise SystemExit("No gamepads detected via inputs.")
        return list(gamepads)

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

    def handle_abs_event_evdev(self, code, value):
        self._raw[code] = value

        if code == EVDEV_CODE_LY:
            self.left_drive = -_normalize_stick(value, self._absinfo.get(code))
            return

        if code == EVDEV_CODE_RY:
            self.right_drive = -_normalize_stick(value, self._absinfo.get(code))
            return

        if code == EVDEV_CODE_LT:
            self._raw["lt"] = _normalize_trigger(value, self._absinfo.get(code))
            self._recompute_aux()
            return

        if code == EVDEV_CODE_RT:
            self._raw["rt"] = _normalize_trigger(value, self._absinfo.get(code))
            self._recompute_aux()
            return

    def handle_abs_event_inputs(self, code, value):
        self._raw[code] = value

        if code == INPUTS_CODE_LY:
            self.left_drive = _normalize_stick(value, INPUTS_ABSINFO.get(code))
            return

        if code == INPUTS_CODE_RY:
            self.right_drive = _normalize_stick(value, INPUTS_ABSINFO.get(code))
            return

        if code == INPUTS_CODE_LT:
            self._raw["lt"] = _normalize_trigger(value, INPUTS_ABSINFO.get(code))
            self._recompute_aux()
            return

        if code == INPUTS_CODE_RT:
            self._raw["rt"] = _normalize_trigger(value, INPUTS_ABSINFO.get(code))
            self._recompute_aux()
            return

    def handle_key_event_evdev(self, code, pressed):
        if code == EVDEV_BTN_LB:
            self._raw["lb"] = 1.0 if pressed else 0.0
            self._recompute_aux()
        elif code == EVDEV_BTN_RB:
            self._raw["rb"] = 1.0 if pressed else 0.0
            self._recompute_aux()

    def handle_key_event_inputs(self, code, pressed):
        if code == INPUTS_BTN_LB:
            self._raw["lb"] = 1.0 if pressed else 0.0
            self._recompute_aux()
        elif code == INPUTS_BTN_RB:
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


def _read_evdev_loop(device, state, stop_event, debug=False, control_queue=None, loop=None):
    try:
        for event in device.read_loop():
            if stop_event.is_set():
                break

            if event.type == ecodes.EV_ABS:
                if debug:
                    print(f"EV_ABS code={event.code} value={event.value}")
                state.handle_abs_event_evdev(event.code, event.value)

            elif event.type == ecodes.EV_KEY:
                # FIX: treat 1 (press) and 2 (hold) as pressed
                pressed = event.value != 0
                if debug and event.code in (EVDEV_BTN_LB, EVDEV_BTN_RB, EVDEV_BTN_A, EVDEV_BTN_B):
                    print(f"EV_KEY code={event.code} value={event.value}")
                state.handle_key_event_evdev(event.code, pressed)
                if control_queue is not None and loop is not None and event.value == 1:
                    if event.code == EVDEV_BTN_B:
                        loop.call_soon_threadsafe(control_queue.put_nowait, "toggle_record")
                    elif event.code == EVDEV_BTN_A:
                        loop.call_soon_threadsafe(control_queue.put_nowait, "play_last")
    except (OSError, ValueError):
        # Device closed or unavailable; exit the reader loop.
        return


def _read_inputs_loop(state, stop_event, debug=False, control_queue=None, loop=None):
    while not stop_event.is_set():
        try:
            events = get_gamepad()
        except (OSError, ValueError):
            if stop_event.is_set():
                break
            continue

        for event in events:
            if stop_event.is_set():
                break
            if event.ev_type == "Absolute":
                if debug:
                    print(f"ABS code={event.code} value={event.state}")
                state.handle_abs_event_inputs(event.code, event.state)
            elif event.ev_type == "Key":
                pressed = event.state != 0
                if debug and event.code in (INPUTS_BTN_LB, INPUTS_BTN_RB, INPUTS_BTN_A, INPUTS_BTN_B):
                    print(f"KEY code={event.code} value={event.state}")
                state.handle_key_event_inputs(event.code, pressed)
                if control_queue is not None and loop is not None and event.state == 1:
                    if event.code == INPUTS_BTN_B:
                        loop.call_soon_threadsafe(control_queue.put_nowait, "toggle_record")
                    elif event.code == INPUTS_BTN_A:
                        loop.call_soon_threadsafe(control_queue.put_nowait, "play_last")


def _default_record_base(repo_root, timestamp):
    return repo_root / "out" / f"teleop-recording-{timestamp}"


def _timestamped_record_base(repo_root, record_out):
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    if not record_out:
        return _default_record_base(repo_root, timestamp)
    path = Path(record_out)
    if not path.is_absolute():
        path = repo_root / path
    return path.with_name(f"{path.name}-{timestamp}")


def _write_recording_csv(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("t,ld,rd,la,ra\n")
        for t, ld, rd, la, ra in samples:
            handle.write(f"{t:.3f},{ld:.3f},{rd:.3f},{la:.3f},{ra:.3f}\n")


def _write_recording_py(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        '"""Auto-generated teleop recording. Deploy this to replay the run."""',
        "",
        "from pybricks.hubs import PrimeHub",
        "from pybricks.parameters import Axis, Button, Direction, Port",
        "from pybricks.pupdevices import Motor",
        "from pybricks.tools import wait",
        "",
        "# PORT MAPPING (confirmed)",
        "DRIVE_LEFT_PORT = Port.D",
        "DRIVE_RIGHT_PORT = Port.C",
        "AUX_LEFT_PORT = Port.F",
        "AUX_RIGHT_PORT = Port.E",
        "",
        "def _dc(motor, value):",
        "    motor.dc(int(value * 100))",
        "",
        "def main():",
        "    hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)",
        "    hub.system.set_stop_button(Button.BLUETOOTH)",
        "",
        "    left_drive = Motor(DRIVE_LEFT_PORT, Direction.COUNTERCLOCKWISE)",
        "    right_drive = Motor(DRIVE_RIGHT_PORT)",
        "    left_aux = Motor(AUX_LEFT_PORT)",
        "    right_aux = Motor(AUX_RIGHT_PORT)",
        "",
        "    sequence = [",
    ]

    last_t = None
    for t, ld, rd, la, ra in samples:
        if last_t is None:
            dt_ms = 0
        else:
            dt_ms = int(round((t - last_t) * 1000))
            if dt_ms < 0:
                dt_ms = 0
        lines.append(f"        ({dt_ms}, {ld:.3f}, {rd:.3f}, {la:.3f}, {ra:.3f}),")
        last_t = t

    lines += [
        "    ]",
        "",
        "    for dt_ms, ld, rd, la, ra in sequence:",
        "        _dc(left_drive, ld)",
        "        _dc(right_drive, rd)",
        "        _dc(left_aux, la)",
        "        _dc(right_aux, ra)",
        "        wait(dt_ms)",
        "",
        "    _dc(left_drive, 0.0)",
        "    _dc(right_drive, 0.0)",
        "    _dc(left_aux, 0.0)",
        "    _dc(right_aux, 0.0)",
        "",
        "if __name__ == '__main__':",
        "    main()",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


class RecordState:
    def __init__(self):
        self.active = False
        self.samples = []
        self.start = None
        self.last_recording_base = None
        self.last_recording_samples = None


async def _replay_samples(samples, bridge, pause_event, debug=False):
    pause_event.set()
    try:
        if not samples:
            return
        if debug:
            print("Replaying last recording...")
        last_t = None
        for t, ld, rd, la, ra in samples:
            if last_t is None:
                dt = 0.0
            else:
                dt = t - last_t
            if dt > 0:
                await asyncio.sleep(dt)
            await bridge.send(ld, rd, la, ra)
            last_t = t
        await bridge.send(0.0, 0.0, 0.0, 0.0)
    finally:
        pause_event.clear()


async def _send_loop(state, bridge, deadband, stop_event, pause_event, debug=False, record_state=None):
    last_send = 0.0
    last = None

    while not stop_event.is_set():
        if pause_event.is_set():
            await asyncio.sleep(POLL_INTERVAL)
            continue

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
            if (
                record_state is not None
                and record_state.active
                and record_state.start is not None
            ):
                record_state.samples.append((now - record_state.start, ld, rd, la, ra))
            last_send = now

            if debug:
                cur = (round(ld, 3), round(rd, 3), round(la, 3), round(ra, 3))
                if last is None or cur != last:
                    print(f"TX ld={cur[0]:.3f} rd={cur[1]:.3f} la={cur[2]:.3f} ra={cur[3]:.3f}")
                    last = cur

        await asyncio.sleep(POLL_INTERVAL / 2)


async def main_async():
    parser = ArgumentParser(description="Deploy teleop and stream Xbox input")
    parser.add_argument("--hub", default=DEFAULT_HUB_NAME, help="Hub name or BLE address")
    parser.add_argument("--device", help="Input device path, e.g. /dev/input/event18")
    parser.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND)
    parser.add_argument("--ble-timeout", type=float, default=DEFAULT_BLE_TIMEOUT)
    parser.add_argument("--no-grab", action="store_true", help="Do not grab exclusive access")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument(
        "--no-record",
        action="store_true",
        help="Disable recording and hub replay generation",
    )
    parser.add_argument(
        "--record-out",
        help="Base path (no extension) for recordings, default: out/teleop-recording-<timestamp>",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    teleop_file = repo_root / "src" / "teleop" / "teleop_hub.py"

    backend = _select_backend()
    devices = _select_devices(backend, args.device)

    if args.debug:
        print(f"Input backend: {backend}")
        for d in devices:
            if backend == "evdev":
                print(f"Using input device: {d.path} ({d.name})")
            else:
                name = getattr(d, "name", "gamepad")
                print(f"Using input device: {name}")

    if backend == "inputs" and args.device:
        print("Warning: --device is ignored when using the inputs backend.")

    state = InputState()
    if backend == "evdev":
        for d in devices:
            state.update_absinfo(d)

    stop_event = asyncio.Event()
    pause_event = asyncio.Event()
    control_queue = asyncio.Queue()

    def _request_stop(*_args):
        stop_event.set()
        if backend == "evdev":
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
        try:
            signal.signal(sig, _request_stop)
        except (ValueError, OSError):
            pass

    grabbed = []
    if backend == "evdev" and not args.no_grab:
        for d in devices:
            try:
                d.grab()
                grabbed.append(d)
            except OSError:
                pass

    record_enabled = not args.no_record
    record_state = RecordState()
    hub = None
    bridge = None

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

        if backend == "evdev":
            readers = [
                asyncio.create_task(
                    asyncio.to_thread(
                        _read_evdev_loop,
                        d,
                        state,
                        stop_event,
                        args.debug,
                        control_queue,
                        loop,
                    )
                )
                for d in devices
            ]
        else:
            readers = [
                asyncio.create_task(
                    asyncio.to_thread(
                        _read_inputs_loop,
                        state,
                        stop_event,
                        args.debug,
                        control_queue,
                        loop,
                    )
                )
            ]
        sender = asyncio.create_task(
            _send_loop(
                state,
                bridge,
                args.deadband,
                stop_event,
                pause_event,
                debug=args.debug,
                record_state=record_state if record_enabled else None,
            )
        )
        async def _save_recording():
            if not record_state.samples:
                print("Recording stopped (no samples captured).")
                return
            record_base = _timestamped_record_base(repo_root, args.record_out)
            _write_recording_csv(record_base.with_suffix(".csv"), record_state.samples)
            _write_recording_py(record_base.with_suffix(".py"), record_state.samples)
            record_state.last_recording_base = record_base
            record_state.last_recording_samples = list(record_state.samples)
            print(f"Saved recording to {record_base.with_suffix('.csv')}")
            print(f"Saved hub replay to {record_base.with_suffix('.py')}")

        async def _control_loop():
            while not stop_event.is_set():
                try:
                    action = await asyncio.wait_for(control_queue.get(), timeout=0.1)
                except asyncio.TimeoutError:
                    continue
                if action == "toggle_record":
                    if not record_enabled:
                        print("Recording disabled; ignoring B toggle.")
                        continue
                    if record_state.active:
                        record_state.active = False
                        record_state.samples.append(
                            (time.monotonic() - record_state.start, 0.0, 0.0, 0.0, 0.0)
                        )
                        record_state.start = None
                        await _save_recording()
                    else:
                        record_state.samples = []
                        record_state.start = time.monotonic()
                        record_state.active = True
                        print("Recording started. Press B to stop.")
                elif action == "play_last":
                    if record_state.active:
                        print("Stop recording before replaying.")
                        continue
                    if not record_state.last_recording_samples:
                        print("No recordings saved yet.")
                        continue
                    await _replay_samples(
                        record_state.last_recording_samples,
                        bridge,
                        pause_event,
                        debug=args.debug,
                    )

        controller = asyncio.create_task(_control_loop())

        await stop_event.wait()

        for r in readers:
            r.cancel()
        sender.cancel()
        controller.cancel()

        try:
            await asyncio.wait_for(
                asyncio.gather(*readers, sender, controller, return_exceptions=True),
                timeout=2.0,
            )
        except asyncio.TimeoutError:
            pass
        if bridge is not None:
            try:
                await asyncio.wait_for(bridge.send(0.0, 0.0, 0.0, 0.0), timeout=1.0)
            except Exception:
                pass
        if hub is not None:
            try:
                await asyncio.wait_for(hub.write_line("quit"), timeout=1.0)
            except Exception:
                pass
    finally:
        if record_enabled and record_state.active:
            record_state.active = False
            record_state.samples.append(
                (time.monotonic() - record_state.start, 0.0, 0.0, 0.0, 0.0)
            )
            record_state.start = None
            if record_state.samples:
                record_base = _timestamped_record_base(repo_root, args.record_out)
                _write_recording_csv(record_base.with_suffix(".csv"), record_state.samples)
                _write_recording_py(record_base.with_suffix(".py"), record_state.samples)
                record_state.last_recording_base = record_base
                record_state.last_recording_samples = list(record_state.samples)
                print(f"Saved recording to {record_base.with_suffix('.csv')}")
                print(f"Saved hub replay to {record_base.with_suffix('.py')}")
        if hub is not None:
            try:
                await asyncio.wait_for(hub.disconnect(), timeout=2.0)
            except Exception:
                pass
        for d in grabbed:
            try:
                d.ungrab()
            except OSError:
                pass
        if backend == "evdev":
            for d in devices:
                try:
                    d.close()
                except OSError:
                    pass


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
