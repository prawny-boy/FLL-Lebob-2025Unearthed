from machine import Pin, PWM, ADC
from time import sleep, ticks_ms, ticks_diff


# Servo helper class encapsulating PWM, pressure sensor and sweep logic
class Servo:
    def __init__(self, servo_pin_num=19, pressure_pin_num=26, freq=50):
        # hardware
        self.servo_pin = Pin(servo_pin_num)
        self.pressure_pin = ADC(pressure_pin_num)
        self.pwm = PWM(self.servo_pin)
        self.pwm.freq(freq)

        # settings
        self.NOT_TOUCH = 25
        self.MAX = 80
        self._min_angle = 0.0
        self._max_angle = 180.0

        # state
        self.current_angle = 0.0  # degrees — last commanded angle

        # cooperative sweep state
        self._sweep_active = False
        self._sweep_speed = 0.0  # degrees per second
        self._sweep_dir = 1      # 1 -> increasing angle, -1 -> decreasing
        self._sweep_min = self._min_angle
        self._sweep_max = self._max_angle
        self._last_update_ms = ticks_ms()

    def set_servo_angle(self, angle, speed=None):
        """Move servo to `angle` (0..180).

        speed: optional float specifying degrees per second. If provided, the
        function will attempt to take abs(target - current_angle) / speed seconds
        to reach the target. If omitted or <=0, the move is applied immediately
        (single duty write) and the function returns (0.0, 0.0).

        Returns (actual_duration_seconds, angular_velocity_deg_per_sec).
        """
        # clamp angle
        angle = max(self._min_angle, min(self._max_angle, float(angle)))

        delta = angle - self.current_angle
        if delta == 0:
            return 0.0, 0.0

        # If a speed is provided, compute duration; otherwise do an immediate set
        if speed is None or speed <= 0:
            # immediate set — no timing information
            self.pwm.duty_u16(self.angle_to_duty(angle))
            self.current_angle = angle
            return 0.0, 0.0

        duration = abs(delta) / float(speed)  # seconds

        # Smooth move by stepping roughly one degree per step for smoothness
        steps = max(1, int(abs(delta)))
        step_angle = delta / steps
        step_delay = duration / steps if duration > 0 else 0
        start_ms = ticks_ms()
        start_angle = self.current_angle
        stopped_early = False
        last_intermediate = self.current_angle
        for i in range(1, steps + 1):
            intermediate = start_angle + step_angle * i
            self.pwm.duty_u16(self.angle_to_duty(intermediate))
            # Check pressure sensor and stop if threshold exceeded
            try:
                if self.pressure_pin.read_u16() >= self.MAX:
                    # stop movement and hold current intermediate
                    last_intermediate = intermediate
                    stopped_early = True
                    break
            except Exception:
                # If the pressure sensor read fails for any reason, ignore and continue
                pass

            last_intermediate = intermediate
            if step_delay > 0:
                sleep(step_delay)

        actual_ms = ticks_diff(ticks_ms(), start_ms)
        actual_duration = actual_ms / 1000.0 if actual_ms >= 0 else duration

        # update tracked angle (if stopped early, set to last_intermediate)
        if stopped_early:
            self.current_angle = last_intermediate
        else:
            self.current_angle = angle

        moved = abs(self.current_angle - start_angle)
        angular_velocity = moved / actual_duration if actual_duration > 0 else float('inf')
        # Return actual duration and measured angular velocity
        return actual_duration, angular_velocity

    def angle_to_duty(self, a):
        """Convert angle (degrees) to PWM duty_u16 value."""
        min_us = 500.0
        max_us = 2400.0
        us = min_us + (max_us - min_us) * a / 180.0
        return int(us * 3.27675)

    def start_sweep(self, speed, direction=1):
        """Start a non-blocking sweep. Call `servo_update()` regularly in your main loop.

        speed: degrees per second (positive). direction: 1 for forward (toward 180),
        -1 for backward (toward 0). Returns True if started, False otherwise.
        """
        try:
            s = float(speed)
        except Exception:
            return False
        if s <= 0:
            return False
        self._sweep_speed = s
        self._sweep_dir = 1 if direction >= 0 else -1
        self._sweep_active = True
        self._last_update_ms = ticks_ms()
        return True

    def stop_sweep(self):
        """Stop any cooperative sweep and hold current position."""
        self._sweep_active = False

    def servo_update(self):
        """Call this frequently from your main loop to advance a cooperative sweep.

        Returns True if an update moved the servo, False otherwise.
        If pressure exceeds MAX the sweep will be stopped and False returned.
        """
        if not self._sweep_active or self._sweep_speed <= 0:
            self._last_update_ms = ticks_ms()
            return False

        now = ticks_ms()
        dt = ticks_diff(now, self._last_update_ms) / 1000.0
        if dt <= 0:
            return False
        self._last_update_ms = now

        # Calculate movement and apply bounds (bounce / reverse)
        move = self._sweep_speed * dt * self._sweep_dir
        new_angle = self.current_angle + move
        if new_angle >= self._sweep_max:
            new_angle = self._sweep_max
            self._sweep_dir = -1
        elif new_angle <= self._sweep_min:
            new_angle = self._sweep_min
            self._sweep_dir = 1

        self.pwm.duty_u16(self.angle_to_duty(new_angle))
        self.current_angle = new_angle

        # pressure sensor check: stop sweep if threshold exceeded
        try:
            if self.pressure_pin.read_u16() >= self.MAX:
                self.stop_sweep()
                return False
        except Exception:
            # ignore sensor read errors
            pass

        return True
    def read_pressure(self):
        return self.pressure_pin.read_u16()*100/65535

# Instantiate and example usage: Sweep the servo cooperatively
servo = Servo()  # default pins
servo.set_servo_angle(0)
state = "forward"
servo.start_sweep(speed=30.0, direction=1)  # 90 degrees per second
while True:
    pressure = servo.read_pressure()
    angle = servo.servo_update()
    print("Pressure: {:.1f}%, Angle: {:.1f} deg".format(pressure, angle))
    if pressure >= servo.MAX:
        servo.stop_sweep()
        break
    sleep(0.02)