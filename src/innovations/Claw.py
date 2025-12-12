import machine
import time

class Force:
    def __init__(self) -> None:
        self.fing_pin = 26
        self.const_pin = 27
        self.max = 65535
        self.min = 13000
        self.force = machine.ADC(machine.Pin(self.fing_pin))
        self.standard = machine.ADC(machine.Pin(self.const_pin))
    def read(self, sensor) -> int:
        force_value = sensor.read_u16()
        return force_value
    def get_percentage(self) -> float:
        force_value = self.read(self.force) - self.read(self.standard)
        percentage = (force_value - self.min) / (self.max - self.min) * 100
        percentage = max(0, min(100, percentage))
        return percentage

class Servo:
    def __init__(self) -> None:
        self.pin = 0
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
diff = 1
start_time = time.ticks_ms()
while True:
    if state == "close":
        servo.set_angle(angle)
        f = force.get_percentage()
        if f >= 70:
            state = "hold"
            print("Closing force reached:", f)
        angle += diff
        if angle >= 180:
            end_time = time.ticks_ms()
            diff -= 2
        if angle <= 0:
            end_time = time.ticks_ms()
            diff += 2
        time.sleep_ms(50)