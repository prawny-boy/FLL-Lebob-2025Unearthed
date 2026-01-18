"""Teleop control using an Xbox controller."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

try:
    from pybricks.iodevices import XboxController
except ImportError as exc:
    raise ImportError(
        "XboxController not available in this Pybricks build. "
        "Update Pybricks or use a supported controller."
    ) from exc

from main import DRIVEBASE_AXLE_TRACK, DRIVEBASE_WHEEL_DIAMETER, DRIVE_PROFILE


POLL_DELAY_MS = 20
DEADBAND = 0.05


def _normalize_trigger(value):
    if value is None:
        return 0.0
    if value < 0:
        value = 0.0
    if value > 1.0:
        if value <= 100.0:
            return value / 100.0
        if value <= 1023.0:
            return value / 1023.0
        return value / 32767.0
    return float(value)


def _normalize_axis(value):
    if value is None:
        return 0.0
    if abs(value) > 1.0:
        if abs(value) <= 100.0:
            return value / 100.0
        if abs(value) <= 1023.0:
            return value / 1023.0
        return value / 32767.0
    return float(value)


def _get_value(controller, names):
    for name in names:
        if hasattr(controller, name):
            value = getattr(controller, name)
            return value() if callable(value) else value
    if hasattr(controller, "axis"):
        for name in names:
            try:
                return controller.axis(name)
            except Exception:
                continue
    return 0.0


def _wait_for_controller(controller):
    if hasattr(controller, "wait_for_connection"):
        controller.wait_for_connection()
        return
    if hasattr(controller, "connect"):
        controller.connect()
    while True:
        connected = None
        if hasattr(controller, "connected"):
            connected = controller.connected() if callable(controller.connected) else controller.connected
        elif hasattr(controller, "is_connected"):
            connected = controller.is_connected()
        if connected is None or connected:
            return
        wait(POLL_DELAY_MS)


def main():
    hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
    hub.system.set_stop_button(Button.BLUETOOTH)

    left_drive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    right_drive = Motor(Port.C)
    drive_base = DriveBase(
        left_drive,
        right_drive,
        DRIVEBASE_WHEEL_DIAMETER,
        DRIVEBASE_AXLE_TRACK,
    )
    drive_base.settings(**DRIVE_PROFILE)

    controller = XboxController()
    _wait_for_controller(controller)

    while True:
        if Button.CENTER in hub.buttons.pressed():
            break

        right_trigger = _normalize_trigger(
            _get_value(controller, ("right_trigger", "rt", "trigger_right", "rtrigger"))
        )
        left_trigger = _normalize_trigger(
            _get_value(controller, ("left_trigger", "lt", "trigger_left", "ltrigger"))
        )
        turn_axis = _normalize_axis(
            _get_value(controller, ("left_x", "lx", "left_stick_x", "stick_left_x"))
        )

        forward = right_trigger - left_trigger
        if abs(forward) < DEADBAND:
            forward = 0.0
        if abs(turn_axis) < DEADBAND:
            turn_axis = 0.0

        speed = forward * DRIVE_PROFILE["straight_speed"]
        turn_rate = turn_axis * DRIVE_PROFILE["turn_rate"]

        if not forward and not turn_axis:
            drive_base.stop()
        else:
            drive_base.drive(speed, turn_rate)

        wait(POLL_DELAY_MS)

    drive_base.stop()


if __name__ == "__main__":
    main()
