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
        self.samples = []
    def read(self, sensor) -> int:
        force_value = sensor.read_u16()
        return force_value
    def get_percentage(self) -> float:
        self.samples.append(self.read(self.standard))
        stand_avg = sum(self.samples)/len(self.samples)
        force_value = self.read(self.force) - int(stand_avg)
        percentage = (force_value) / (self.max - force_value) * 100
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
state = "close"
angle = 0.0
diff = 5.0
start_time = time.ticks_ms()
while True:
    if state == "close":
        servo.set_angle(angle)
        f = force.read(force.force)
        if f >= 10000:
            state = "hold"
            print("Closing force reached:", f)
        angle += diff
        if angle >= 180:
            end_time = time.ticks_ms()
            diff = -diff
        if angle <= 0:
            end_time = time.ticks_ms()
            diff = abs(diff)
        time.sleep_ms(50)
        print(f)