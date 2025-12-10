import machine
import time

class Force:
    def __init__(self) -> None:
        self.pin = 27
        self.max = 65535
        self.min = 13000
        self.force = machine.ADC(machine.Pin(self.pin))
    def read(self) -> int:
        force_value = self.force.read_u16()
        return force_value
    def get_percentage(self) -> float:
        force_value = self.read()
        percentage = (force_value - self.min) / (self.max - self.min) * 100
        percentage = max(0, min(100, percentage))
        return percentage

class Servo:
    def __init__(self) -> None:
        self.pin = 19
        self.deg0 = 1638
        self.deg180 = 8191
        self.servo = machine.PWM(machine.Pin(self.pin))
        self.servo.freq(50)
    def set_angle(self, angle) -> None:
        write = 6553/180*angle+self.deg0
        self.servo.duty_u16(int(write))
        time.sleep_ms(20)


servo = Servo()
force = Force()
vals = []
state = "close"
angle = 0.0
start_time = time.ticks_ms()
while True:
    if state == "close":
        servo.set_angle(angle)
        f = force.get_percentage()
        if f >= 70:
            state = "hold"
            print("Closing force reached:", f)
        angle += 1
        if angle > 180:
            end_time = time.ticks_ms()
            break
duration = end_time - start_time
print("Claw closed. Duration (ms):", duration)