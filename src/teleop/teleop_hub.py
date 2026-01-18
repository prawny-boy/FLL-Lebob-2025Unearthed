"""Teleop receiver for PC input over Pybricks BLE stdin (tank + aux)."""

try:
    import sys
except ImportError:
    import usys as sys

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import Matrix


# PORT MAPPING (confirmed)
DRIVE_LEFT_PORT = Port.D
DRIVE_RIGHT_PORT = Port.C
AUX_LEFT_PORT = Port.F
AUX_RIGHT_PORT = Port.E

RUNNING_ANIMATION = tuple(
    Matrix(frame)
    for frame in (
        [
            [0, 0, 100, 100, 100],
            [100, 100, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 100, 100, 0, 0],
        ],
        [
            [100, 0, 0, 100, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 100, 0, 0, 100],
        ],
        [
            [100, 100, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 100, 100],
        ],
        [
            [100, 100, 100, 0, 0],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [0, 0, 100, 100, 100],
        ],
        [
            [100, 100, 100, 100, 0],
            [100, 0, 0, 0, 0],
            [100, 0, 0, 0, 100],
            [0, 0, 0, 0, 100],
            [0, 100, 100, 100, 100],
        ],
        [
            [100, 100, 100, 100, 100],
            [100, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 100],
            [0, 0, 0, 0, 100],
        ],
        [
            [100, 100, 100, 100, 100],
            [0, 0, 0, 0, 100],
            [0, 0, 0, 0, 0],
            [100, 0, 0, 0, 0],
            [100, 100, 100, 100, 100],
        ],
        [
            [0, 100, 100, 100, 100],
            [0, 0, 0, 0, 100],
            [0, 0, 0, 0, 0],
            [100, 0, 0, 0, 0],
            [100, 100, 100, 100, 100],
        ],
    )
)

RECORD_ON = Matrix(
    [
        [100, 100, 100, 100, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 100, 100, 100, 100],
    ]
)
RECORD_OFF = Matrix([[0, 0, 0, 0, 0]] * 5)
REPLAY_ICON = Matrix(
    [
        [100, 0, 0, 0, 100],
        [0, 100, 0, 100, 0],
        [0, 0, 100, 0, 0],
        [0, 100, 0, 100, 0],
        [100, 0, 0, 0, 100],
    ]
)


def _clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def _parse_line(line):
    line = line.strip()
    if not line:
        return None
    if line.lower() in ("q", "quit", "exit"):
        return "quit"
    if line.lower().startswith("mode:"):
        return ("mode", line.split(":", 1)[1].strip().lower())
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


def _dc(motor, value):
    motor.dc(int(value * 100))


def main():
    hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
    hub.system.set_stop_button(Button.BLUETOOTH)

    left_drive = Motor(DRIVE_LEFT_PORT, Direction.COUNTERCLOCKWISE)
    right_drive = Motor(DRIVE_RIGHT_PORT)

    left_aux = Motor(AUX_LEFT_PORT)
    right_aux = Motor(AUX_RIGHT_PORT)

    hub.display.animate(RUNNING_ANIMATION, 30)

    print("Hub teleop started")

    count = 0
    while True:
        line = sys.stdin.readline()
        if not line:
            _dc(left_drive, 0.0)
            _dc(right_drive, 0.0)
            _dc(left_aux, 0.0)
            _dc(right_aux, 0.0)
            continue

        parsed = _parse_line(line)
        if parsed == "quit":
            break
        if parsed is None:
            continue
        if isinstance(parsed, tuple) and parsed[0] == "mode":
            mode = parsed[1]
            if mode == "record":
                hub.display.animate((RECORD_ON, RECORD_OFF), 300)
            elif mode == "replay":
                hub.display.icon(REPLAY_ICON)
            else:
                hub.display.animate(RUNNING_ANIMATION, 30)
            continue

        ld, rd, la, ra = parsed

        _dc(left_drive, ld)
        _dc(right_drive, rd)
        _dc(left_aux, la)
        _dc(right_aux, ra)

        count += 1
        if count % 25 == 0:
            print(f"RX ld={ld:.2f} rd={rd:.2f} la={la:.2f} ra={ra:.2f}")

    _dc(left_drive, 0.0)
    _dc(right_drive, 0.0)
    _dc(left_aux, 0.0)
    _dc(right_aux, 0.0)


if __name__ == "__main__":
    main()
