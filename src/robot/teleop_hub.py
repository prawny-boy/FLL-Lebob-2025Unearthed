"""Teleop receiver for PC input over Pybricks BLE stdin (tank + aux)."""

try:
    import sys
except ImportError:
    import usys as sys

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Port
from pybricks.pupdevices import Motor


# PORT MAPPING (confirmed)
DRIVE_LEFT_PORT = Port.D
DRIVE_RIGHT_PORT = Port.C
AUX_LEFT_PORT = Port.F
AUX_RIGHT_PORT = Port.E


def _clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def _parse_line(line):
    line = line.strip()
    if not line:
        return None
    if line.lower() in ("q", "quit", "exit"):
        return "quit"
    try:
        ld, rd, la, ra = map(float, line.split(",", 3))
    except ValueError:
        return None
    return (
        _clamp(ld),
        _clamp(rd),
        _clamp(la),
        _clamp(ra),
    )


def main():
    hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
    hub.system.set_stop_button(Button.BLUETOOTH)

    left_drive = Motor(DRIVE_LEFT_PORT, Direction.COUNTERCLOCKWISE)
    right_drive = Motor(DRIVE_RIGHT_PORT)

    left_aux = Motor(AUX_LEFT_PORT)
    right_aux = Motor(AUX_RIGHT_PORT)

    print("Hub teleop started")

    count = 0
    while True:
        line = sys.stdin.readline()
        if not line:
            left_drive.stop()
            right_drive.stop()
            left_aux.stop()
            right_aux.stop()
            continue

        parsed = _parse_line(line)
        if parsed == "quit":
            break
        if parsed is None:
            continue

        ld, rd, la, ra = parsed

        left_drive.dc(int(ld * 100))
        right_drive.dc(int(rd * 100))
        left_aux.dc(int(la * 100))
        right_aux.dc(int(ra * 100))

        count += 1
        if count % 25 == 0:
            print(f"RX ld={ld:.2f} rd={rd:.2f} la={la:.2f} ra={ra:.2f}")

    left_drive.stop()
    right_drive.stop()
    left_aux.stop()
    right_aux.stop()


if __name__ == "__main__":
    main()
