import machine

class servo:
    def __init__(self) -> None:
        servoPin = 19
        servoMach = machine.PWM