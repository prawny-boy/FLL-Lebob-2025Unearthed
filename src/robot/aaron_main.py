"""PrimeHub entry point for the FLL Unearthed 2025 robot."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import Matrix, StopWatch, hub_menu
from pybricks.tools import wait as sleep

DRIVEBASE_WHEEL_DIAMETER = 88  # 56 is small, 88 is big
DRIVEBASE_AXLE_TRACK = 115
LOW_VOLTAGE = 7200
HIGH_VOLTAGE = 8400
DRIVE_PROFILE = {
    "straight_speed": 500,
    "straight_acceleration": 750,
    "turn_rate": 300,
    "turn_acceleration": 500,
}
DEFAULT_SETTLE_DELAY = 250
ROBOT_MAX_TORQUE = 700

class PIDController:
    """Basic PID helper reused by the smart drive and turn helpers."""

    def __init__(
        self,
        k_p,
        k_i,
        k_d,
        loop_delay_time=0.02,
        integral_limit=None,
        output_limit=None,
    ):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.loop_delay_time = loop_delay_time
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0
        self.previous_error = 0

    def calculate(self, error):
        self.integral += error * self.loop_delay_time
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        derivative = (error - self.previous_error) / self.loop_delay_time
        output = self.k_p * error + self.k_i * self.integral + self.k_d * derivative
        if self.output_limit is not None:
            output = max(-self.output_limit, min(output, self.output_limit))
        self.previous_error = error
        return output


class Robot:
    """Wrapper around the PrimeHub, drive base, and attachments."""

    def __init__(self, use_gyro=False, drive_profile=None):
        profile = DRIVE_PROFILE.copy()
        if drive_profile:
            profile.update(drive_profile)
        self.drive_profile = profile

        self.left_big = Motor(Port.C)
        self.right_big = Motor(Port.D)
        self.left_drive = Motor(Port.E)
        self.right_drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.big_motors = {"left": self.left_big, "right": self.right_big}
        self.drive_motors = (self.left_drive, self.right_drive)

        self.drive_base = DriveBase(
            self.left_drive,
            self.right_drive,
            DRIVEBASE_WHEEL_DIAMETER,
            DRIVEBASE_AXLE_TRACK,
        )
        self.drive_base.settings(**DRIVE_PROFILE)
        # self.drive_base.settings((500, 750, 300, 500))

        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        self.hub.system.set_stop_button(Button.BLUETOOTH)
        self.hub.imu.reset_heading(0)
        self.drive_base.use_gyro(use_gyro=use_gyro)

    def _resolve_speed(self, speed, profile_key):
        return speed if speed is not None else self.drive_profile[profile_key]

    def _stop_drivebase(self, then):
        if then == Stop.COAST:
            self.drive_base.stop()
        else:
            self.drive_base.brake()

    def _override_drive_settings(self, **overrides):
        valid = {}
        for key, value in overrides.items():
            if value is None or key not in self.drive_profile:
                continue
            valid[key] = abs(value)
        if not valid:
            return False
        settings = self.drive_profile.copy()
        settings.update(valid)
        self.drive_base.settings(**settings)
        return True

    def _restore_drive_settings(self):
        self.drive_base.settings(**self.drive_profile)

    def rotate_attachment(self, side, degrees, speed=None, then=Stop.BRAKE, wait=True):
        motor = self.big_motors.get(side)
        if motor is None:
            raise ValueError("Attachment side must be 'left' or 'right'")
        resolved_speed = self._resolve_speed(speed, "turn_rate")
        motor.run_angle(resolved_speed, degrees, then, wait)

    def rotate_attachment_until_stalled(
        self, side, speed=None, then=Stop.BRAKE, duty_limit=50
    ):
        motor = self.big_motors.get(side)
        if motor is None:
            raise ValueError("Attachment side must be 'left' or 'right'")
        resolved_speed = self._resolve_speed(speed, "turn_rate")
        motor.run_until_stalled(resolved_speed, then, duty_limit)

    def rotate_right_motor(self, degrees, speed=None, then=Stop.BRAKE, wait=True):
        self.rotate_attachment("right", degrees, speed, then, wait)

    def rotate_left_motor(self, degrees, speed=None, then=Stop.BRAKE, wait=True):
        self.rotate_attachment("left", degrees, speed, then, wait)

    def rotate_right_motor_until_stalled(
        self, speed=None, then=Stop.COAST, duty_limit=50
    ):
        self.rotate_attachment_until_stalled("right", speed, then, duty_limit)
        self.rotate_right_motor(-10, then=Stop.COAST, wait=False)

    def rotate_left_motor_until_stalled(
        self, speed=None, then=Stop.COAST, duty_limit=20
    ):
        self.rotate_attachment_until_stalled("left", speed, then, duty_limit)

    def wrap_angle(self, angle):
        return (angle + 180) % 360 - 180

    def drive_for_distance(
        self,
        distance,
        then=Stop.BRAKE,
        wait=True,
        settle_time=DEFAULT_SETTLE_DELAY,
        speed=None,
    ):
        if not distance:
            return
        overrides_applied = False
        if speed is not None:
            overrides_applied = self._override_drive_settings(
                straight_speed=speed
            )
        try:
            self.drive_base.straight(-distance, then, wait)
            # distance because backwards
        finally:
            if overrides_applied:
                self._restore_drive_settings()
        if settle_time:
            sleep(settle_time)

    def smart_drive_for_distance(
        self,
        distance,
        speed=None,
        then=Stop.BRAKE,
        k_p=2.5,
        k_i=0.01,
        k_d=0.2,
        loop_delay_time=0.02,
    ):
        distance = -distance
        if not distance:
            return
        resolved_speed = self._resolve_speed(speed, "straight_speed")
        pid = PIDController(k_p, k_i, k_d, loop_delay_time)
        target_heading = self.hub.imu.heading()
        self.drive_base.reset()
        direction = 1 if distance >= 0 else -1
        while abs(self.drive_base.distance()) < abs(distance):
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            correction = pid.calculate(error)
            self.drive_base.drive(direction * resolved_speed, -correction)
            sleep(loop_delay_time)
        self._stop_drivebase(then)

    def turn_in_place(
        self, degrees, then=Stop.BRAKE, settle_time=DEFAULT_SETTLE_DELAY, speed=None
    ):
        adjusted = degrees * 1.25
        overrides_applied = False
        if speed is not None:
            overrides_applied = self._override_drive_settings(turn_rate=speed)
        self.hub.imu.reset_heading(0)
        try:
            self.drive_base.turn(-adjusted, Stop.COAST, True)
        finally:
            if overrides_applied:
                self._restore_drive_settings()
        self._stop_drivebase(then)
        if settle_time:
            sleep(settle_time)

    def smart_turn_in_place(
        self,
        target_angle,
        then=Stop.HOLD,
        k_p=3.5,
        k_i=0.02,
        k_d=0.3,
        loop_delay_time=0.02,
        speed=None,
    ):
        turn_limit = abs(speed) if speed is not None else 400
        pid = PIDController(k_p, k_i, k_d, loop_delay_time, output_limit=turn_limit)
        target_heading = self.wrap_angle(self.hub.imu.heading() + target_angle)
        self.drive_base.stop()
        # smart_turn_in_place_iterations = 0
        while True:
            # smart_turn_in_place_iterations += 1
            # if smart_turn_in_place_iterations > 100:
            #     break

            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < 2.0:
                break
            correction = pid.calculate(error)
            self.drive_base.drive(0, -correction)
            # print(
            #     "Heading: {:.2f} Error: {:.2f} Corr: {:.2f}".format(
            #         current_heading, error, correction
            #     )
            # )
            sleep(loop_delay_time)
        self._stop_drivebase(then)

    def curve(self, radius, angle, then=Stop.COAST, wait=True, speed=None):
        overrides_applied = False
        if speed is not None:
            overrides_applied = self._override_drive_settings(
                straight_speed=speed
            )
        try:
            self.drive_base.curve(radius, angle, then, wait)
        finally:
            if overrides_applied:
                self._restore_drive_settings()

    def status_light(self, color):
        self.hub.light.off()
        if color is not None:
            self.hub.light.on(color)

    def battery_display(self):
        voltage = self.hub.battery.voltage()
        # pct = rescale(voltage, LOW_VOLTAGE, HIGH_VOLTAGE, 1, 100)
        print(f"Battery %: {None} Voltage: {voltage}")

        # self.status_light(color)
        # return color

    def clean_motors(self):
        for motor in self.drive_motors + (self.left_big,):
            motor.run_angle(999, 1000, wait=False)
        # Ensure the last attachment waits before returning so the hub
        # does not start another mission mid-cleaning.
        self.right_big.run_angle(999, 1000)

def run_1(r:Robot):
    "Sweep and map"
    # Position arms
    r.rotate_right_motor(-90, then=Stop.COAST)
    r.rotate_left_motor(140, then=Stop.COAST, wait=False)
    sleep(800)
    r.rotate_right_motor(-25, then=Stop.COAST, wait=False)
    r.rotate_left_motor(-90)

    # ALIGN THE 
    # Sweep
    r.hub.imu.reset_heading(0)
    r.drive_for_distance(644) # Drive up to the sweep box
    r.smart_turn_in_place(-90) # Face thing
    r.drive_for_distance(20)
    r.smart_turn_in_place(-30) # Sweep left
    r.smart_turn_in_place(60, speed=200) # Right
    r.smart_turn_in_place(-30) # Back to middle
    sleep(200) # Wait for robot to stop moving
    r.drive_for_distance(-110) # Drive back
    r.rotate_left_motor(90, then=Stop.COAST, wait=False) # Arm down
    sleep(500)
    r.drive_for_distance(100, then=Stop.COAST, speed=300) # Drive forward to align
    r.smart_turn_in_place(-r.hub.imu.heading() - 90)
    sleep(500) # Wait for align
    r.drive_for_distance(-50, speed=200) # Drive back
    sleep(2000) # Wait for brush to stop moving
    r.drive_for_distance(50, speed=200) # Drive forward to pick up the brush
    r.rotate_left_motor(-90, wait=False) # Pick up brush (hopefully), move arm up, yeet into the oval
    sleep(500)
    r.drive_for_distance(-30) # Drive back
    sleep(1000)

    # Map
    r.smart_turn_in_place(50) # Turn to face map
    r.rotate_left_motor(88, then=Stop.COAST, wait=False) # Arm down
    r.drive_for_distance(180) # Push map section
    sleep(300)
    r.drive_for_distance(-45)
    r.rotate_left_motor(-90, wait=False) # Arm up
    r.drive_for_distance(57) # Move to position for sliding the map
    r.smart_turn_in_place(8)
    r.rotate_left_motor(85, then=Stop.COAST, wait=False) # Arm down onto the slide map
    sleep(500)
    r.drive_for_distance(64) # Slide the map

    # Post map alignment
    r.rotate_left_motor(-140, then=Stop.COAST, wait=False) # Move long arm out the way
    r.drive_for_distance(-227)
    r.smart_turn_in_place(33)
    r.drive_for_distance(500, then=Stop.COAST, speed=250) # Go to wall to align

def run_2(r:Robot):
    "Statue and minecart"
    # Minecart
    r.drive_for_distance(-220) # Drive back from wall
    r.smart_turn_in_place(90) # Get in position for minecart
    r.drive_for_distance(150)

    # Detour for the statue
    r.rotate_right_motor(-40, then=Stop.COAST, wait=False) # Move short arm onto ground
    r.drive_for_distance(204) # Arrive at statue
    r.drive_for_distance(-10)
    sleep(1000)
    r.rotate_right_motor(90, speed=500) # Push statue up
    r.drive_for_distance(-215)

    # Back to the minecart
    r.rotate_right_motor(-93, then=Stop.COAST)
    r.smart_turn_in_place(-90) # Face minecart
    r.drive_for_distance(50) # Arrive at minecart
    r.rotate_right_motor(56, speed=50) # Push up minecart
    sleep(1000) # Wait for minecart to roll down
    r.drive_for_distance(-120)

def run_3(r:Robot):
    "Align"
    # Post minecart alignment
    r.smart_turn_in_place(90)
    r.drive_for_distance(140)
    r.smart_turn_in_place(-28)
    r.drive_for_distance(400)

    # Align arms
    r.rotate_right_motor(-40, then=Stop.COAST)
    r.rotate_right_motor(-30, then=Stop.COAST, wait=False)
    r.rotate_left_motor(140, then=Stop.COAST, wait=False)
    sleep(800)
    r.rotate_right_motor(-35, then=Stop.COAST, wait=False)
    sleep(800)
    r.rotate_left_motor(-135, then=Stop.COAST)
    sleep(1500)

    # Post minecart alignment finish
    r.smart_turn_in_place(-62)
    r.drive_for_distance(250, then=Stop.COAST, wait=False)
    sleep(400)

def run_4(r:Robot):
    # Platform
    r.drive_for_distance(-10) # Back up to give space
    r.smart_turn_in_place(90) # Move left a little
    r.drive_for_distance(40) # Drive left
    r.smart_turn_in_place(45) # Turn to face platform
    r.rotate_right_motor(90) # Put shovel on ground (short right arm)
    r.drive_for_distance(450, then=Stop.COAST, wait=False) # Push up the statue
    sleep(1400)
    r.drive_for_distance(-150) # Go back out
    r.rotate_right_motor(-90) # Flatten right motor

    # Bucket
    r.smart_turn_in_place(7) # Face bucket
    r.rotate_left_motor(130, then=Stop.COAST, wait=False) # Push down the bucket
    sleep(1500)
    r.rotate_left_motor(-135) # Arm up again
    r.smart_turn_in_place(38)
    r.drive_for_distance(-300, then=Stop.COAST, speed=200, wait=False) # Align against wall

def run_5(r:Robot):
    "Post bucket alignment, no points, important"
    # Post bucket alignment
    r.rotate_right_motor(40)
    sleep(1000)

    r.drive_for_distance(65)
    r.smart_turn_in_place(90)
    r.drive_for_distance(300, then=Stop.COAST, speed=250, wait=False)
    sleep(4000)
    r.drive_for_distance(-280)
    r.smart_turn_in_place(-90)
    r.drive_for_distance(-120, then=Stop.COAST, wait=False)
    sleep(1600)

def run_6(r:Robot):
    # Platform and boulders
    r.rotate_right_motor(-75, then=Stop.COAST, wait=False)
    r.drive_for_distance(250)
    r.smart_turn_in_place(-45)
    r.rotate_right_motor(40, then=Stop.COAST, wait=False)
    r.smart_turn_in_place(-45)
    r.drive_for_distance(300)
    r.rotate_right_motor(-30, then=Stop.COAST, wait=False)
    r.rotate_left_motor(140, then=Stop.COAST, wait=False)
    sleep(800)
    r.rotate_right_motor(-25, then=Stop.COAST, wait=False)
    sleep(1800)
    r.rotate_left_motor(-135, then=Stop.COAST)
    r.drive_for_distance(100)
    r.smart_turn_in_place(-70)
    r.smart_turn_in_place(70)
    r.drive_for_distance(-150)
    r.smart_turn_in_place(90)
    r.drive_for_distance(100)
    r.smart_turn_in_place(180)

r = Robot()
# run_1(r)
# run_2(r)
# run_3(r)
# run_4(r)
# run_5(r)
# run_6(r)