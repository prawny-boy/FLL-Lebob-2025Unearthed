"""PrimeHub entry point for the FLL Unearthed 2025 robot."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import Matrix, StopWatch, hub_menu
from pybricks.tools import wait as sleep

DRIVEBASE_WHEEL_DIAMETER = 88  # 56 is small, 88 is big
DRIVEBASE_AXLE_TRACK = 115
LOW_VOLTAGE = 7000
HIGH_VOLTAGE = 8300
MENU_OPTIONS = ("1", "2", "3", "4", "5", "6", "7", "8", "C")
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


class Robot:
    """Wrapper around the PrimeHub, drive base, and attachments."""

    def __init__(self, use_gyro=False, drive_profile=None):
        drive_profile = drive_profile or DRIVE_PROFILE
        self.drive_profile = {
            "straight_speed": drive_profile["straight_speed"],
            "straight_acceleration": drive_profile["straight_acceleration"],
            "turn_rate": drive_profile["turn_rate"],
            "turn_acceleration": drive_profile["turn_acceleration"],
        }

        self.left_drive = Motor(Port.C)
        self.right_drive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
        self.right_big = Motor(Port.A)
        self.left_big = Motor(Port.B)

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

    def rotate_right_motor(self, degrees, speed=None, then=Stop.BRAKE, wait=True):
        if speed is None:
            speed = self.drive_profile["turn_rate"]
        self.right_big.run_angle(speed, degrees, then, wait)

    def rotate_left_motor(self, degrees, speed=None, then=Stop.BRAKE, wait=True):
        if speed is None:
            speed = self.drive_profile["turn_rate"]
        self.left_big.run_angle(speed, degrees, then, wait)

    def rotate_right_motor_until_stalled(
        self, speed=None, then=Stop.COAST, duty_limit=50
    ):
        if speed is None:
            speed = self.drive_profile["turn_rate"]
        self.right_big.run_until_stalled(speed, then, duty_limit)

    def rotate_left_motor_until_stalled(
        self, speed=None, then=Stop.COAST, duty_limit=20
    ):
        if speed is None:
            speed = self.drive_profile["turn_rate"]
        self.left_big.run_until_stalled(speed, then, duty_limit)

    def wrap_angle(self, angle):
        return (angle + 180) % 360 - 180

    def drive_for_distance(
        self, distance, then=Stop.BRAKE, wait=True, settle_time=DEFAULT_SETTLE_DELAY
    ):
        self.drive_base.straight(distance, then, wait)
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
        if speed is None:
            speed = self.drive_profile["straight_speed"]
        integral = 0
        previous_error = 0
        target_heading = self.hub.imu.heading()
        self.drive_base.reset()
        while abs(self.drive_base.distance()) < abs(distance):
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            proportional = error
            integral += error * loop_delay_time
            derivative = (error - previous_error) / loop_delay_time
            correction = k_p * proportional + k_i * integral + k_d * derivative
            self.drive_base.drive(speed, -correction)
            previous_error = error
            sleep(loop_delay_time)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        elif then == Stop.COAST:
            self.drive_base.stop()

    def turn_in_place(self, degrees, then=Stop.BRAKE, settle_time=DEFAULT_SETTLE_DELAY):
        adjusted = degrees * 1.25
        self.hub.imu.reset_heading(0)
        self.drive_base.turn(-adjusted, Stop.COAST, True)
        if then == Stop.COAST:
            self.drive_base.stop()
        else:
            self.drive_base.brake()
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
    ):
        integral = 0
        previous_error = 0
        target_heading = self.wrap_angle(self.hub.imu.heading() + target_angle)
        self.drive_base.stop()
        while True:
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < 2.0:
                break
            proportional = error
            integral += error * loop_delay_time
            derivative = (error - previous_error) / loop_delay_time
            correction = k_p * proportional + k_i * integral + k_d * derivative
            self.drive_base.drive(0, -correction)
            previous_error = error
            print(
                "Heading: {:.2f} Error: {:.2f} Corr: {:.2f}".format(
                    current_heading, error, correction
                )
            )
            sleep(loop_delay_time)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        elif then == Stop.COAST:
            self.drive_base.stop()

    def curve(self, radius, angle, then=Stop.COAST, wait=True):
        self.drive_base.curve(radius, angle, then, wait)

    def status_light(self, color):
        self.hub.light.off()
        self.hub.light.on(color)

    def battery_display(self):
        voltage = self.hub.battery.voltage()
        pct = rescale(voltage, LOW_VOLTAGE, HIGH_VOLTAGE, 1, 100)
        print("Battery %: {} Voltage: {}".format(round(pct, 1), voltage))
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
        self.left_drive.run_angle(999, 1000, wait=False)
        self.right_drive.run_angle(999, 1000, wait=False)
        self.left_big.run_angle(999, 1000, wait=False)
        self.right_big.run_angle(999, 1000)


def mission_function_one(robot):
    robot.rotate_left_motor_until_stalled(-100)
    robot.rotate_right_motor_until_stalled(-100)
    robot.drive_for_distance(755)
    robot.turn_in_place(-90)
    robot.drive_for_distance(65)
    robot.rotate_left_motor(98)
    robot.turn_in_place(35)
    robot.turn_in_place(-70)
    robot.turn_in_place(35)
    robot.drive_for_distance(-95)
    robot.turn_in_place(5)
    robot.drive_for_distance(120)
    robot.rotate_left_motor(-115)
    robot.drive_for_distance(-110)
    robot.turn_in_place(150)
    robot.drive_for_distance(165)
    robot.rotate_right_motor_until_stalled(50)
    robot.turn_in_place(-55)
    robot.rotate_right_motor(-55)
    sleep(2000)
    robot.rotate_right_motor_until_stalled(100)
    robot.drive_for_distance(-100)
    robot.turn_in_place(50)
    robot.rotate_right_motor_until_stalled(-100)
    robot.drive_for_distance(-200)
    robot.turn_in_place(-45)
    robot.drive_for_distance(-700)


def mission_function_two(robot):
    robot.drive_for_distance(200)
    robot.turn_in_place(90)
    robot.drive_for_distance(400)
    robot.rotate_right_motor_until_stalled(200)
    robot.drive_for_distance(-50)
    robot.rotate_right_motor(-120)
    robot.drive_for_distance(100)
    robot.drive_for_distance(-50)
    robot.turn_in_place(-45)
    robot.drive_for_distance(100)
    robot.turn_in_place(45)
    robot.drive_for_distance(800)


def mission_function_three(robot):
    robot.rotate_left_motor_until_stalled(100)
    robot.drive_for_distance(710)
    robot.turn_in_place(30)
    robot.drive_for_distance(30)
    robot.turn_in_place(-110)
    robot.drive_for_distance(200)


def mission_function_four(robot):
    robot.drive_for_distance(200)
    robot.turn_in_place(-45)
    robot.drive_for_distance(300, wait=False)
    robot.rotate_left_motor_until_stalled(-100)
    robot.rotate_right_motor_until_stalled(-100)
    sleep(500)
    robot.rotate_left_motor(120, wait=False)
    robot.rotate_right_motor(120, speed=500)
    robot.rotate_right_motor(-120)
    robot.drive_for_distance(-700)


def mission_function_five(robot):
    robot.drive_for_distance(100)
    robot.turn_in_place(-30)
    robot.rotate_left_motor_until_stalled(100)
    robot.drive_for_distance(900)
    robot.rotate_left_motor(-120, speed=500, wait=False)
    sleep(200)
    robot.drive_for_distance(50)


def mission_function_six(robot):
    robot.smart_turn_in_place(90)
    robot.smart_turn_in_place(90)
    robot.smart_turn_in_place(90)
    robot.smart_turn_in_place(90)
    robot.drive_for_distance(1000)


def mission_function_seven(robot):
    pass


def mission_function_eight(robot):
    pass


MISSIONS = {
    "1": mission_function_one,
    "2": mission_function_two,
    "3": mission_function_three,
    "4": mission_function_four,
    "5": mission_function_five,
    "6": mission_function_six,
    "7": mission_function_seven,
    "8": mission_function_eight,
}


def rescale(value, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        raise ValueError("in_max and in_min must be different values")
    if value < in_min:
        value = in_min
    elif value > in_max:
        value = in_max
    scale = (value - in_min) / (in_max - in_min)
    return out_min + scale * (out_max - out_min)


class MissionControl:
    def __init__(self, robot, missions=None, menu_options=MENU_OPTIONS):
        self.robot = robot
        self.missions = missions if missions is not None else MISSIONS
        self.menu_options = menu_options
        self.stopwatch = StopWatch()
        self.battery_status = Color.GREEN
        self.last_run = "C"

    def _start_stopwatch(self):
        self.stopwatch.reset()

    def _build_menu(self):
        start_index = (self.menu_options.index(self.last_run) + 1) % len(
            self.menu_options
        )
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
        self._start_stopwatch()
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
            selection = hub_menu(*self._build_menu())
            if selection == "C":
                self.robot.clean_motors()
                continue
            self.last_run = self._execute_mission(selection)


def main():
    MissionControl(Robot(use_gyro=False)).run()


main()
