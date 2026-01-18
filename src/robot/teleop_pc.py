"""Read Xbox controller input on PC and send it to a Pybricks hub."""

import argparse
import asyncio
import signal
import time

import evdev
from evdev import ecodes
from bleak import BleakClient, BleakScanner

from pybricksdev.ble import find_device
from pybricksdev.ble.pybricks import Command, PYBRICKS_COMMAND_EVENT_UUID, PYBRICKS_SERVICE_UUID


POLL_INTERVAL = 0.02
DEFAULT_DEADBAND = 0.05
DEFAULT_BLE_TIMEOUT = 10.0
DEFAULT_BLE_RETRIES = 5
DEFAULT_HUB_ID = "auto"


ABS_LX_CODES = [getattr(ecodes, name) for name in ("ABS_X",) if hasattr(ecodes, name)]
ABS_LT_CODES = [
    getattr(ecodes, name)
    for name in ("ABS_Z", "ABS_BRAKE", "ABS_LT")
    if hasattr(ecodes, name)
]
ABS_RT_CODES = [
    getattr(ecodes, name)
    for name in ("ABS_RZ", "ABS_GAS", "ABS_RT")
    if hasattr(ecodes, name)
]


class InputState:
    def __init__(self):
        self.forward = 0.0
        self.turn = 0.0
        self._raw = {}
        self._absinfo = {}

    def update_absinfo(self, device):
        caps = device.capabilities(absinfo=True).get(ecodes.EV_ABS, [])
        for code, absinfo in caps:
            self._absinfo[code] = absinfo

    def handle_abs_event(self, code, value):
        self._raw[code] = value
        if code in ABS_LX_CODES:
            self.turn = _normalize_stick(value, self._absinfo.get(code))
        elif code in ABS_LT_CODES:
            self._raw["lt"] = _normalize_trigger(value, self._absinfo.get(code))
        elif code in ABS_RT_CODES:
            self._raw["rt"] = _normalize_trigger(value, self._absinfo.get(code))

        lt = self._raw.get("lt", 0.0)
        rt = self._raw.get("rt", 0.0)
        self.forward = rt - lt


class StdinBridge:
    def __init__(self, client):
        self.client = client

    async def send(self, forward, turn):
        payload = f"{forward:.3f},{turn:.3f}\n".encode("ascii")
        data = bytes([Command.WRITE_STDIN]) + payload
        await self.client.write_gatt_char(PYBRICKS_COMMAND_EVENT_UUID, data, response=True)


def _normalize_trigger(value, absinfo):
    if absinfo is None:
        return 0.0
    span = absinfo.max - absinfo.min
    if not span:
        return 0.0
    return max(0.0, min(1.0, (value - absinfo.min) / span))


def _normalize_stick(value, absinfo):
    if absinfo is None:
        return 0.0
    span = max(abs(absinfo.min), abs(absinfo.max))
    if not span:
        return 0.0
    return max(-1.0, min(1.0, value / span))


def _select_device(device_path=None):
    if device_path:
        return evdev.InputDevice(device_path)

    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if "xbox" in device.name.lower():
            return device
    for device in devices:
        if "gamepad" in device.name.lower():
            return device
    if devices:
        return devices[0]
    raise SystemExit("No input devices found.")


async def _send_loop(state, bridge, deadband, stop_event):
    last_send = 0.0
    while not stop_event.is_set():
        now = time.monotonic()
        if now - last_send >= POLL_INTERVAL:
            forward = state.forward
            turn = state.turn
            if abs(forward) < deadband:
                forward = 0.0
            if abs(turn) < deadband:
                turn = 0.0
            await bridge.send(forward, turn)
            last_send = now
        await asyncio.sleep(POLL_INTERVAL / 2)


async def _read_loop(device, state, stop_event):
    async for event in device.async_read_loop():
        if stop_event.is_set():
            break
        if event.type == ecodes.EV_ABS:
            state.handle_abs_event(event.code, event.value)


async def main_async():
    parser = argparse.ArgumentParser(description="Xbox controller teleop bridge")
    parser.add_argument(
        "--hub",
        dest="hub_name",
        default=DEFAULT_HUB_ID,
        help="Hub name, BLE address, or 'auto' (default: %(default)s)",
    )
    parser.add_argument("--device", help="Input device path, e.g. /dev/input/event3")
    parser.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND)
    parser.add_argument(
        "--ble-timeout",
        type=float,
        default=DEFAULT_BLE_TIMEOUT,
        help="BLE scan timeout per attempt in seconds",
    )
    parser.add_argument(
        "--ble-retries",
        type=int,
        default=DEFAULT_BLE_RETRIES,
        help="Number of BLE scan retries (0 = retry forever)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print debug status updates",
    )
    parser.add_argument(
        "--no-grab",
        action="store_true",
        help="Do not grab exclusive access to the controller device",
    )
    args = parser.parse_args()

    device = _select_device(args.device)
    if args.debug:
        print(f"Using input device: {device.path} ({device.name})")
    state = InputState()
    state.update_absinfo(device)

    stop_event = asyncio.Event()

    def _request_stop(*_args):
        stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_stop)

    grabbed = False
    if not args.no_grab:
        try:
            device.grab()
            grabbed = True
        except OSError:
            grabbed = False
    try:
        ble_device = None
        retries = args.ble_retries
        hub_name = None if args.hub_name == "auto" else args.hub_name
        if args.debug:
            print(f"Scanning for hub '{hub_name or 'auto'}'...")
        attempt = 0
        while True:
            attempt += 1
            try:
                if args.debug:
                    devices = await BleakScanner.discover(timeout=2.0)
                    if devices:
                        print("Nearby BLE devices:")
                        for dev in devices:
                            print(f"  {dev.name or 'Unknown'} {dev.address}")
                    else:
                        print("No BLE devices found in scan.")
                if hub_name and ":" in hub_name:
                    ble_device = await BleakScanner.find_device_by_address(
                        hub_name, timeout=args.ble_timeout
                    )
                elif hub_name:
                    ble_device = await find_device(
                        name=hub_name, timeout=args.ble_timeout
                    )
                else:
                    ble_device = await find_device(timeout=args.ble_timeout)
                if ble_device is None:
                    pybricks_devices = await BleakScanner.discover(
                        timeout=args.ble_timeout, service_uuids=[PYBRICKS_SERVICE_UUID]
                    )
                    if pybricks_devices:
                        if args.debug:
                            print("Pybricks BLE devices:")
                            for dev in pybricks_devices:
                                print(f"  {dev.name or 'Unknown'} {dev.address}")
                        if len(pybricks_devices) == 1:
                            ble_device = pybricks_devices[0]
                if ble_device is None:
                    raise asyncio.TimeoutError
                break
            except asyncio.TimeoutError:
                if args.debug:
                    print(f"BLE scan timed out (attempt {attempt}), retrying...")
                if retries > 0 and attempt >= retries:
                    break
                await asyncio.sleep(1)
        if ble_device is None:
            raise asyncio.TimeoutError("Hub not found over BLE.")
        if args.debug:
            print(f"Found hub: {ble_device.name} ({ble_device.address})")
        async with BleakClient(ble_device) as client:
            if args.debug:
                print("Connected to hub, starting control stream.")
            bridge = StdinBridge(client)
            reader = asyncio.create_task(_read_loop(device, state, stop_event))
            sender = asyncio.create_task(_send_loop(state, bridge, args.deadband, stop_event))
            await stop_event.wait()
            reader.cancel()
            sender.cancel()
            await asyncio.gather(reader, sender, return_exceptions=True)
            if args.debug:
                print("Stopping control stream.")
            await bridge.send(0.0, 0.0)
            await bridge.send(0.0, 0.0)
    finally:
        if grabbed:
            device.ungrab()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
