"""Standalone square test: forward, square, back to start."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch
from pybricks.tools import wait as sleep


DRIVEBASE_WHEEL_DIAMETER = 62.4
DRIVEBASE_AXLE_TRACK = 150

SIDE_LENGTH = 300


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


class PIDController:
    def __init__(self, k_p, k_i, k_d, integral_limit=None, output_limit=None):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.has_previous = False

    def calculate(self, error, dt):
        if dt <= 0:
            dt = 0.001
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        if self.has_previous:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
            self.has_previous = True
        output = (self.k_p * error) + (self.k_i * self.integral) + (self.k_d * derivative)
        if self.output_limit is not None:
            output = clamp(output, -self.output_limit, self.output_limit)
        self.previous_error = error
        return output


def wrap_angle(angle):
    return (angle + 180) % 360 - 180


def better_brake(drive_base, left_drive, right_drive, timeout_ms=350):
    drive_base.stop()
    left_pid = PIDController(0.55, 0.0, 0.05, integral_limit=20, output_limit=160)
    right_pid = PIDController(0.55, 0.0, 0.05, integral_limit=20, output_limit=160)
    left_drive.reset_angle()
    right_drive.reset_angle()
    sw = StopWatch()
    last_ms = sw.time()
    while sw.time() <= timeout_ms:
        now = sw.time()
        dt = (now - last_ms) / 1000.0
        last_ms = now
        left_error = left_drive.angle()
        right_error = right_drive.angle()
        if abs(left_error) <= 1.0 and abs(right_error) <= 1.0:
            break
        left_drive.run(-left_pid.calculate(left_error, dt))
        right_drive.run(-right_pid.calculate(right_error, dt))
        sleep(10)
    left_drive.brake()
    right_drive.brake()


def drive_for_distance(hub, drive_base, left_drive, right_drive, distance):
    speed = 350
    angle_pid = PIDController(2.5, 0.05, 0.3, integral_limit=60, output_limit=250)
    dist_pid = PIDController(1.2, 0.0, 0.05, output_limit=abs(speed))

    target_heading = hub.imu.heading()
    drive_base.stop()
    start_distance = drive_base.distance()

    direction = 1 if distance >= 0 else -1
    sw = StopWatch()
    last_ms = sw.time()

    while True:
        now = sw.time()
        dt = (now - last_ms) / 1000.0
        last_ms = now

        traveled = drive_base.distance() - start_distance
        distance_error = distance - traveled

        if abs(distance_error) <= 5:
            break

        heading_error = wrap_angle(target_heading - hub.imu.heading())
        turn_correction = angle_pid.calculate(heading_error, dt)

        if abs(distance_error) > 150:
            linear_speed = direction * abs(speed)
        else:
            linear_speed = dist_pid.calculate(distance_error, dt)
            if abs(linear_speed) < 40:
                linear_speed = direction * 40
            linear_speed = clamp(linear_speed, -abs(speed), abs(speed))

        drive_base.drive(linear_speed, turn_correction)

        if sw.time() > 7000:
            break

        sleep(10)

    # Hold heading while decelerating
    drive_base.stop()
    angle_pid.reset()
    hold_sw = StopWatch()
    hold_last = hold_sw.time()
    while hold_sw.time() < 300:
        now = hold_sw.time()
        dt = (now - hold_last) / 1000.0
        hold_last = now
        heading_error = wrap_angle(target_heading - hub.imu.heading())
        if abs(heading_error) <= 0.5:
            break
        correction = angle_pid.calculate(heading_error, dt)
        drive_base.drive(0, correction)
        sleep(10)

    better_brake(drive_base, left_drive, right_drive)
    sleep(100)


def turn_in_place(hub, drive_base, left_drive, right_drive, degrees):
    turn_limit = 500 if abs(degrees) >= 45 else 360
    minimum_turn_rate = max(turn_limit * 0.06, 10)
    fine_tune_turn_rate = max(turn_limit * 0.03, 6)

    pid = PIDController(1.6, 0.0, 0.2, integral_limit=50, output_limit=turn_limit)

    target_heading = wrap_angle(hub.imu.heading() + degrees)
    drive_base.stop()

    sw = StopWatch()
    last_ms = sw.time()
    prev_error = wrap_angle(target_heading - hub.imu.heading())
    consecutive_hits = 0

    while True:
        now = sw.time()
        dt = (now - last_ms) / 1000.0
        last_ms = now

        error = wrap_angle(target_heading - hub.imu.heading())

        if abs(error) < 0.15:
            consecutive_hits += 1
            if consecutive_hits >= 2:
                break
        else:
            consecutive_hits = 0

        if error * prev_error < 0 and abs(error) < 0.3:
            break

        correction = pid.calculate(error, dt)

        error_mag = abs(error)
        if error_mag < 1.0:
            effective_limit = max(turn_limit * 0.12, 28)
            min_rate = fine_tune_turn_rate
        elif error_mag < 5:
            effective_limit = max(turn_limit * 0.28, 110)
            min_rate = minimum_turn_rate
        elif error_mag < 12:
            effective_limit = max(turn_limit * 0.5, 200)
            min_rate = minimum_turn_rate
        else:
            effective_limit = turn_limit
            min_rate = minimum_turn_rate

        correction = clamp(correction, -effective_limit, effective_limit)

        if error_mag < 12 and abs(correction) < min_rate:
            correction = min_rate if error > 0 else -min_rate

        drive_base.drive(0, correction)
        prev_error = error

        if sw.time() > 4000:
            break

        sleep(10)

    pid.reset()
    hold_limit = min(turn_limit, 24)
    settle_min_rate = max(hold_limit * 0.35, 6)

    settle_sw = StopWatch()
    settle_last = settle_sw.time()
    while settle_sw.time() < 120:
        now = settle_sw.time()
        dt = (now - settle_last) / 1000.0
        settle_last = now

        error = wrap_angle(target_heading - hub.imu.heading())

        if abs(error) <= 0.075:
            break

        correction = pid.calculate(error, dt)
        correction = clamp(correction, -hold_limit, hold_limit)

        if abs(correction) < settle_min_rate and abs(error) > 0.05:
            correction = settle_min_rate if error > 0 else -settle_min_rate

        drive_base.drive(0, correction)
        sleep(10)

    drive_base.drive(0, 0)
    sleep(50)
    drive_base.stop()
    sleep(100)


def main():
    hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
    hub.system.set_stop_button(Button.BLUETOOTH)

    left_drive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    right_drive = Motor(Port.C)

    drive_base = DriveBase(
        left_drive, right_drive, DRIVEBASE_WHEEL_DIAMETER, DRIVEBASE_AXLE_TRACK
    )
    drive_base.use_gyro(True)
    drive_base.settings(
        straight_speed=350,
        straight_acceleration=600,
        turn_rate=250,
        turn_acceleration=400,
    )

    drive_base.stop()
    sleep(50)
    hub.imu.reset_heading(0)
    sleep(50)

    print("Square test: forward, square, back")

    # Drive forward one side length
    drive_base.straight(SIDE_LENGTH)
    sleep(100)
    print("After initial drive: {}".format(hub.imu.heading()))

    # Drive a full square (4 sides + 4 turns)
    for i in range(4):
        drive_base.straight(SIDE_LENGTH)
        sleep(100)
        print("After drive {}: {}".format(i + 1, hub.imu.heading()))
        turn_in_place(hub, drive_base, left_drive, right_drive, 90)
        print("After turn {}: {} (expected {})".format(i + 1, hub.imu.heading(), (i + 1) * 90))

    # Drive backward to starting position
    drive_base.straight(-SIDE_LENGTH)
    sleep(100)

    print("Final heading: {} (should be ~360)".format(hub.imu.heading()))
    print("Done!")


if __name__ == "__main__":
    main()
