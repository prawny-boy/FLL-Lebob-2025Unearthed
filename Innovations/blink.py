from machine import Pin, PWM, ADC
from time import sleep


class PressureMotor:

    def __init__(
        self,
        motor_pin_num: int = 19,
        pressure_pin_num: int = 26,
        pwm_freq: int = 50,
        target_percent: float = 90.0,
        run_duty: int = 7000,
    ) -> None:
        self.motor_pin = Pin(motor_pin_num)
        self.motor_pwm = PWM(self.motor_pin)
        self.motor_pwm.freq(pwm_freq)

        self.pressure_adc = ADC(pressure_pin_num)

        self.target_percent = float(target_percent)

        self.run_duty = max(0, min(65535, int(run_duty)))

    def _set_motor_duty(self, duty: int) -> None:
        duty = max(0, min(65535, int(duty)))
        self.motor_pwm.duty_u16(duty)

    def start_motor(self) -> None:
        self._set_motor_duty(self.run_duty)

    def stop_motor(self) -> None:
        self._set_motor_duty(0)

    def read_pressure_percent(self) -> float:
        raw = self.pressure_adc.read_u16()
        return raw * 100.0 / 65535.0


def _angle_to_duty(angle: float) -> int:
    angle = max(0.0, min(180.0, float(angle)))
    min_us = 500.0
    max_us = 2400.0
    us = min_us + (max_us - min_us) * angle / 180.0
    return int(us * 65535.0 / 20000.0)


def _read_pressure_percent(adc: ADC) -> float:
    raw = adc.read_u16()
    return raw * 100.0 / 65535.0


def main() -> None:
    servo_pin = Pin(19)
    pwm = PWM(servo_pin)
    pwm.freq(50)

    pressure_adc = ADC(26)
    target_percent = 80.0

    print(
        "[Innovations] SG90 + pressure: sweeping until {:.1f}%, then hold.".format(
            target_percent
        )
    )

    angle = 0.0
    direction = 1

    while True:
        pwm.duty_u16(_angle_to_duty(angle))
        sleep(0.05)

        pressure = _read_pressure_percent(pressure_adc)
        print("Angle: {:5.1f}°, Pressure: {:5.1f}%".format(angle, pressure))

        if pressure >= target_percent:
            print(
                "[Innovations] Target pressure reached ({:.1f}%). Holding at {:.1f}°.".format(
                    pressure, angle
                )
            )
            break

        angle += direction * 5.0

        if angle >= 180.0:
            angle = 180.0
            direction = -1
        elif angle <= 0.0:
            angle = 0.0
            direction = 1

    hold_duty = _angle_to_duty(angle)
    pwm.duty_u16(hold_duty)
    print("[Innovations] Holding position. Servo will stay at {:.1f}°.".format(angle))

    while True:
        pressure = _read_pressure_percent(pressure_adc)
        print("Holding: Angle {:5.1f}°, Pressure {:5.1f}%".format(angle, pressure))
        sleep(0.2)


if __name__ == "__main__":
    main()
