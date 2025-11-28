from machine import Pin, PWM, ADC
from time import sleep


"""Simple pressure‑based motor controller for the Innovations rig.

Behavior requested:
    • Spin the motor continuously until the pressure sensor reaches
      about 90% of its full range.
    • Then stop the motor and keep it stopped ("hold").

This file is designed to be run directly on the controller used for the
Innovations attachment, separate from the main FLL robot code.
"""


class PressureMotor:
    """Drive a motor until a pressure threshold is reached, then hold.

    Assumptions (adjust to match your wiring):
      - `motor_pin_num` controls a DC motor or continuous‑rotation servo via PWM.
      - `pressure_pin_num` is connected to an analog pressure sensor.
      - Pressure is read using `ADC.read_u16()` and interpreted as 0–100%.
    """

    def __init__(
        self,
        motor_pin_num: int = 19,
        pressure_pin_num: int = 26,
        # For standard / continuous-rotation servos, use ~50 Hz PWM.
        pwm_freq: int = 50,
        target_percent: float = 90.0,
        # Duty value tuned for continuous spinning on a servo at 50 Hz.
        # This corresponds to roughly a 2 ms pulse width (full speed).
        run_duty: int = 7000,
    ) -> None:
        # Motor output (PWM)
        self.motor_pin = Pin(motor_pin_num)
        self.motor_pwm = PWM(self.motor_pin)
        self.motor_pwm.freq(pwm_freq)

        # Pressure sensor input (ADC)
        self.pressure_adc = ADC(pressure_pin_num)

        # Target pressure threshold in percent (0.0–100.0)
        self.target_percent = float(target_percent)

        # Duty value to use while running the motor (0–65535)
        self.run_duty = max(0, min(65535, int(run_duty)))

    # --- Low‑level helpers -------------------------------------------------

    def _set_motor_duty(self, duty: int) -> None:
        """Apply a raw duty value to the motor PWM (0–65535)."""
        duty = max(0, min(65535, int(duty)))
        self.motor_pwm.duty_u16(duty)

    def start_motor(self) -> None:
        """Start spinning the motor at the configured duty."""
        self._set_motor_duty(self.run_duty)

    def stop_motor(self) -> None:
        """Stop the motor (no drive)."""
        self._set_motor_duty(0)

    def read_pressure_percent(self) -> float:
        """Read pressure as a 0–100% float using the 16‑bit ADC value."""
        raw = self.pressure_adc.read_u16()  # 0–65535
        return raw * 100.0 / 65535.0


def _angle_to_duty(angle: float) -> int:
    """Convert an angle (0–180) to a duty_u16 value for an SG90 servo.

    SG90 expects ~0.5 ms (0°) to ~2.4 ms (180°) pulses at 50 Hz (20 ms period).
    """

    angle = max(0.0, min(180.0, float(angle)))
    min_us = 500.0
    max_us = 2400.0
    us = min_us + (max_us - min_us) * angle / 180.0
    # duty_u16 is proportional to pulse width within the 20 ms period.
    return int(us * 65535.0 / 20000.0)


def _read_pressure_percent(adc: ADC) -> float:
    """Read pressure from the given ADC as a 0–100% value."""

    raw = adc.read_u16()  # 0–65535
    return raw * 100.0 / 65535.0


def main() -> None:
    """Sweep servo back and forth until ~80% pressure, then stop and hold.

    - SG90 positional servo on pin 19 sweeps between 0° and 180°.
    - Pressure sensor on ADC 26 is read each step.
    - When pressure reaches about 80%, the servo stops where it is and
      continues holding that angle.
    """

    # Servo setup on pin 19
    servo_pin = Pin(19)
    pwm = PWM(servo_pin)
    pwm.freq(50)  # SG90 uses ~50 Hz

    # Pressure sensor on ADC 26
    pressure_adc = ADC(26)
    target_percent = 80.0

    print(
        "[Innovations] SG90 + pressure: sweeping until {:.1f}%, then hold.".format(
            target_percent
        )
    )

    # --- Phase 1: sweep back and forth until we hit target pressure -------

    angle = 0.0
    direction = 1  # 1 = increasing toward 180, -1 = decreasing toward 0

    while True:
        # Move servo to current angle
        pwm.duty_u16(_angle_to_duty(angle))
        sleep(0.05)

        # Read pressure
        pressure = _read_pressure_percent(pressure_adc)
        print("Angle: {:5.1f}°, Pressure: {:5.1f}%".format(angle, pressure))

        # Check threshold
        if pressure >= target_percent:
            print(
                "[Innovations] Target pressure reached ({:.1f}%). Holding at {:.1f}°.".format(
                    pressure, angle
                )
            )
            break

        # Step angle for next iteration
        angle += direction * 5.0

        # Bounce at the ends of the 0–180° range
        if angle >= 180.0:
            angle = 180.0
            direction = -1
        elif angle <= 0.0:
            angle = 0.0
            direction = 1

    # --- Phase 2: hold at the last angle, keep monitoring pressure --------

    hold_duty = _angle_to_duty(angle)
    pwm.duty_u16(hold_duty)
    print("[Innovations] Holding position. Servo will stay at {:.1f}°.".format(angle))

    while True:
        pressure = _read_pressure_percent(pressure_adc)
        print("Holding: Angle {:5.1f}°, Pressure {:5.1f}%".format(angle, pressure))
        sleep(0.2)


if __name__ == "__main__":
    main()
