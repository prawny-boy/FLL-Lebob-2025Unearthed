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
RUNNING_ANIMATION = tuple(
    Matrix(frame)
    for frame in (
        [
            [0, 0, 100, 100, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 100, 100, 0, 0],
        ],
        [
            [100, 0, 0, 100, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 100, 0, 0, 100],
        ],
        [
            [100, 100, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 100, 100],
        ],
        [
            [100, 100, 100, 0, 0],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [0, 0, 100, 100, 100],
        ],
        [
            [100, 100, 100, 100, 0],
            [100, 0, 0, 0, 0],
            [100, 0, 0, 0, 100],
            [0, 0, 0, 0, 100],
            [0, 100, 100, 100, 100],
        ],
        [
            [100, 100, 100, 100, 100],
            [100, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 100],
            [0, 0, 0, 0, 100],
        ],
        [
            [100, 100, 100, 100, 100],
            [0, 0, 0, 0, 100],
            [0, 0, 0, 0, 0],
            [100, 0, 0, 0, 0],
            [100, 100, 100, 100, 100],
        ],
        [
            [0, 100, 100, 100, 100],
            [0, 0, 0, 0, 100],
            [100, 0, 0, 0, 100],
            [100, 0, 0, 0, 0],
            [100, 100, 100, 100, 0],
        ],
    )
)
MISSION_REGISTRY = {}


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

        self.left_drive = Motor(Port.C)
        self.right_drive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
        self.right_big = Motor(Port.A)
        self.left_big = Motor(Port.B)
        self.big_motors = {"left": self.left_big, "right": self.right_big}
        self.drive_motors = (self.left_drive, self.right_drive)

        self.drive_base = DriveBase(
            self.left_drive,
            self.right_drive,
            DRIVEBASE_WHEEL_DIAMETER,
            DRIVEBASE_AXLE_TRACK,
        )
        self.drive_base.settings(**self.drive_profile)

        self.hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
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
        self, side, speed=None, then=Stop.COAST, duty_limit=50
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
            self.drive_base.straight(distance, then, wait)
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
        then=Stop.BRAKE,
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
        while True:
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < 2.0:
                break
            correction = pid.calculate(error)
            self.drive_base.drive(0, -correction)
            print(
                "Heading: {:.2f} Error: {:.2f} Corr: {:.2f}".format(
                    current_heading, error, correction
                )
            )
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
        pct = rescale(voltage, LOW_VOLTAGE, HIGH_VOLTAGE, 1, 100)
        print(f"Battery %: {round(pct, 1)} Voltage: {voltage}")
        if pct < 40:
            print("EMERGENCY: BATTERY LOW!")
            color = Color.RED
        elif pct < 70:
            print("Battery is below 70% Please charge!")
            color = Color.YELLOW
        else:
            color = Color.GREEN
        self.status_light(color)
        return color

    def clean_motors(self):
        for motor in self.drive_motors + (self.left_big,):
            motor.run_angle(999, 1000, wait=False)
        # Ensure the last attachment waits before returning so the hub
        # does not start another mission mid-cleaning.
        self.right_big.run_angle(999, 1000)


class MissionControl:
    def __init__(self, robot:Robot, missions=None, menu_options_override=None):
        self.robot = robot
        self.missions = missions if missions is not None else MISSION_REGISTRY
        if menu_options_override is not None:
            self.menu_options = list(menu_options_override)
        else:
            # Build default menu from registered missions (sorted by key) and include "C" (clean)
            # Expect mission keys like "1","2","A","B", etc.
            default_slots = sorted(self.missions.keys())
            # Put "C" (clean motors) first for convenience; add if not already present
            self.menu_options = ["C"] + [s for s in default_slots if s != "C"]
        # Common initialization
        self.stopwatch = StopWatch()
        self.battery_status = Color.GREEN
        # Ensure last_run is a valid entry
        self.last_run = "C" if "C" in self.menu_options else (self.menu_options[0] if self.menu_options else None)

    def _build_menu(self):
        # Defensive: if menu_options somehow empty, return an empty list (hub_menu should handle or caller should guard)
        if not self.menu_options:
            return []
        try:
            start_index = (self.menu_options.index(self.last_run) + 1) % len(
                self.menu_options
            )
        except (ValueError, TypeError):
            start_index = 0
        return [
            self.menu_options[(start_index + i) % len(self.menu_options)]
            for i in range(len(self.menu_options))
        ]

    def _execute_mission(self, selection):
        mission = self.missions.get(selection)
        if mission is None:
            print("Mission slot {} is unassigned.".format(selection))
            return self.last_run
        self.robot.status_light(Color.YELLOW)
        self.robot.hub.display.animate(RUNNING_ANIMATION, 30)
        print("Running #{}...".format(selection))
        self.stopwatch.reset()
        self.robot.drive_for_distance(-10, settle_time=0)
        self.robot.hub.imu.reset_heading(0)
        mission(self.robot)
        elapsed = self.stopwatch.time()
        print("Done running #{} in {}ms".format(selection, elapsed))
        self.robot.status_light(self.battery_status)
        return selection

    def run(self):
        self.battery_status = self.robot.battery_display()
        while True:
            menu = self._build_menu()
            if not menu:
                print("No menu options available.")
                return
            selection = hub_menu(*menu)
            if selection == "C":
                self.robot.clean_motors()
                continue
            self.last_run = self._execute_mission(selection)


def mission(slot):
    """Decorator used to register mission handlers."""

    def decorator(func):
        MISSION_REGISTRY[slot] = func
        return func

    return decorator


@mission("1")
def mission_function_one(robot:Robot):
    robot.rotate_left_motor_until_stalled(-100)
    robot.rotate_right_motor_until_stalled(-100)
    robot.drive_for_distance(745)
    robot.turn_in_place(-90)
    robot.drive_for_distance(65)
    robot.drive_for_distance(-30)
    robot.rotate_left_motor_until_stalled(100)
    robot.rotate_left_motor(-45)
    robot.turn_in_place(35)
    robot.turn_in_place(-70)
    robot.turn_in_place(32)
    robot.drive_for_distance(-95)
    robot.turn_in_place(10)
    robot.drive_for_distance(120)
    robot.rotate_left_motor(-115)
    robot.drive_for_distance(-110)
    robot.turn_in_place(90)
    robot.drive_for_distance(-700)
    # robot.rotate_right_motor_until_stalled(50)
    # robot.turn_in_place(-55)
    # robot.rotate_right_motor(-55)
    # sleep(2000)
    # robot.rotate_right_motor_until_stalled(100)
    # robot.drive_for_distance(-100)
    # robot.turn_in_place(50)
    # robot.rotate_right_motor_until_stalled(-100)
    # robot.drive_for_distance(-200)
    # robot.turn_in_place(-45)
    # robot.drive_for_distance(-700)


@mission("2")
def mission_function_two(robot:Robot):
    robot.rotate_right_motor_until_stalled(-100)
    robot.drive_for_distance(190)
    robot.smart_turn_in_place(90)
    robot.drive_for_distance(385)
    robot.rotate_right_motor_until_stalled(100)
    robot.drive_for_distance(-70)
    robot.drive_for_distance(25)
    robot.rotate_right_motor_until_stalled(-100)
    robot.drive_for_distance(200)
    robot.drive_for_distance(-100)
    robot.turn_in_place(-45)
    robot.drive_for_distance(200)
    robot.turn_in_place(45)
    robot.drive_for_distance(1250)


@mission("3")
def mission_function_three(robot:Robot):
    robot.rotate_left_motor_until_stalled(100)
    robot.drive_for_distance(30)
    robot.smart_turn_in_place(-15)
    robot.drive_for_distance(690)
    robot.turn_in_place(50)
    robot.drive_for_distance(65) # into the boulders
    robot.turn_in_place(-75) # does boulders and uncovering artifact
    robot.drive_for_distance(-200)
    robot.turn_in_place(-45)
    robot.drive_for_distance(200)
    robot.turn_in_place(20)
    robot.rotate_left_motor_until_stalled(-200, then=Stop.HOLD)
    robot.rotate_left_motor(30)
    robot.drive_for_distance(-400, speed=250, wait=False)
    sleep(300)
    robot.rotate_left_motor(45)
    robot.rotate_left_motor(30)
    robot.drive_for_distance(100)
    robot.rotate_left_motor(100)
    robot.drive_for_distance(-700)


@mission("4")
def mission_function_four(robot:Robot):
    robot.drive_for_distance(30)
    robot.smart_turn_in_place(-15)
    robot.drive_for_distance(490)
    robot.turn_in_place(-120)
    robot.drive_for_distance(200)
    robot.smart_turn_in_place(-45)
    robot.drive_for_distance(140)
    robot.turn_in_place(90)
    robot.drive_for_distance(305)
    robot.turn_in_place(90)
    robot.drive_for_distance(95, speed=100)
    robot.drive_for_distance(-100)
    robot.turn_in_place(-90)
    robot.drive_for_distance(-850)


@mission("5")
def mission_function_five(robot:Robot):
    robot.drive_for_distance(300)
    robot.smart_turn_in_place(-90)
    robot.drive_for_distance(855)
    robot.turn_in_place(37)
    robot.rotate_right_motor_until_stalled(100)
    robot.drive_for_distance(135)
    robot.rotate_right_motor(-100)
    robot.rotate_left_motor(-90, speed=150)
    robot.rotate_left_motor(90)


@mission("6")
def mission_function_six(robot:Robot):
    robot.smart_turn_in_place(90)
    robot.smart_turn_in_place(90)
    robot.smart_turn_in_place(90)
    robot.smart_turn_in_place(90)
    robot.drive_for_distance(1000)


@mission("7")
def mission_function_seven(robot:Robot):
    # identical to mission 3, but using smart functions where applicable
    robot.rotate_left_motor_until_stalled(100)
    robot.smart_drive_for_distance(30)
    robot.smart_turn_in_place(-15)
    robot.smart_drive_for_distance(680)
    robot.smart_turn_in_place(50)
    robot.smart_drive_for_distance(75)  # into the boulders
    robot.smart_turn_in_place(-75)  # does boulders and uncovering artifact
    robot.smart_drive_for_distance(-200)
    robot.smart_turn_in_place(-45)
    robot.smart_drive_for_distance(200)
    robot.smart_turn_in_place(20)
    robot.rotate_left_motor_until_stalled(-200, then=Stop.HOLD)
    robot.rotate_left_motor(30)
    robot.smart_drive_for_distance(-400, speed=250)
    sleep(500)
    robot.rotate_left_motor(45, speed=100)
    robot.rotate_left_motor(30, speed=100)
    robot.smart_drive_for_distance(100)
    robot.rotate_left_motor(100)
    robot.smart_drive_for_distance(-600)


@mission("8")
def mission_function_eight(robot:Robot):
    pass


def rescale(value, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        raise ValueError("in_max and in_min must be different values")
    if value < in_min:
        value = in_min
    elif value > in_max:
        value = in_max
    scale = (value - in_min) / (in_max - in_min)
    return out_min + scale * (out_max - out_min)


def main():
    MissionControl(Robot(use_gyro=False)).run()


main()
