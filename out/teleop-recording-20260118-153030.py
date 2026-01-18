"""Auto-generated teleop recording. Deploy this to replay the run."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import wait

# PORT MAPPING (confirmed)
DRIVE_LEFT_PORT = Port.D
DRIVE_RIGHT_PORT = Port.C
AUX_LEFT_PORT = Port.F
AUX_RIGHT_PORT = Port.E

def _dc(motor, value):
    motor.dc(int(value * 100))

def main():
    hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
    hub.system.set_stop_button(Button.BLUETOOTH)

    left_drive = Motor(DRIVE_LEFT_PORT, Direction.COUNTERCLOCKWISE)
    right_drive = Motor(DRIVE_RIGHT_PORT)
    left_aux = Motor(AUX_LEFT_PORT)
    right_aux = Motor(AUX_RIGHT_PORT)

    sequence = [
        (0, 0.000, 0.000, 0.000, 0.000),
        (89, 0.000, 0.000, 0.000, 0.000),
        (91, 0.000, 0.000, 0.000, 0.000),
        (90, 0.000, 0.000, 0.000, 0.000),
        (88, 0.000, 0.000, 0.000, 0.000),
        (92, 0.000, 0.000, 0.000, 0.000),
        (90, 0.000, 0.000, 0.000, 0.000),
        (90, 0.000, 0.000, 0.000, 0.000),
        (90, 0.000, 0.000, 0.000, 0.000),
        (90, 0.077, 0.000, 0.000, 0.000),
        (90, 0.731, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (89, 1.000, 0.000, 0.000, 0.000),
        (91, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (89, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (89, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 0.574, 0.000, 0.000, 0.000),
        (91, 0.285, 0.000, 0.000, 0.000),
        (89, 0.000, 0.000, 0.000, 0.000),
        (90, 0.261, 0.000, 0.000, 0.000),
        (90, 1.000, 0.000, 0.000, 0.000),
        (90, 0.000, 0.000, 0.000, 0.000),
        (90, 0.000, 0.000, 0.000, 0.000),
        (91, 0.000, 0.000, 0.000, 0.000),
        (118, 0.000, 0.000, 0.000, 0.000),
    ]

    for dt_ms, ld, rd, la, ra in sequence:
        _dc(left_drive, ld)
        _dc(right_drive, rd)
        _dc(left_aux, la)
        _dc(right_aux, ra)
        wait(dt_ms)

    _dc(left_drive, 0.0)
    _dc(right_drive, 0.0)
    _dc(left_aux, 0.0)
    _dc(right_aux, 0.0)

if __name__ == '__main__':
    main()
