"""Simple on-hub menu for manual attachment control."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.tools import hub_menu, wait


# Tune these if the attachments feel too slow/fast for manual nudging.
ATTACHMENT_SPEED = 400
POLL_DELAY_MS = 50


hub = PrimeHub()
# Match attachment mapping from main.py: right_big -> Port E, left_big -> Port F.
left_big = Motor(Port.F)
right_big = Motor(Port.E)

w
def _wait_for_button_release():
    """Prevent a held selection button from immediately exiting the loop."""
    while hub.buttons.pressed():
        wait(POLL_DELAY_MS)


def _manual_motor_control(motor: Motor):
    _wait_for_button_release()
    while True:
        pressed = set(hub.buttons.pressed())
        if Button.LEFT in pressed and Button.RIGHT not in pressed:
            motor.run(-ATTACHMENT_SPEED)
        elif Button.RIGHT in pressed and Button.LEFT not in pressed:
            motor.run(ATTACHMENT_SPEED)
        else:
            motor.stop()
        if Button.CENTER in pressed:
            motor.stop(Stop.HOLD)
            _wait_for_button_release()
            break
        wait(POLL_DELAY_MS)


def main():
    selection = hub_menu("L", "R")
    if selection == "L":
        _manual_motor_control(left_big)
    elif selection == "R":
        _manual_motor_control(right_big)


if __name__ == "__main__":
    main()
