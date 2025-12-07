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

    def rotate_right_motor(self, degrees, speed=300, then=Stop.BRAKE, wait=True):
        self.right_big.run_angle(speed, degrees, then, wait)

    def rotate_left_motor(self, degrees, speed=300, then=Stop.BRAKE, wait=True):
        self.left_big.run_angle(speed, degrees, then, wait)

    def rotate_right_motor_until_stalled(
        self, speed, then=Stop.COAST, duty_limit=50
    ):
        self.right_big.run_until_stalled(speed, then, duty_limit)

    def rotate_left_motor_until_stalled(
        self, speed, then=Stop.COAST, duty_limit=20
    ):
        self.left_big.run_until_stalled(speed, then, duty_limit)

    def wrap_angle(self, angle):
        return (angle + 180) % 360 - 180

    def drive_for_distance(
        self,
        distance,
        then=Stop.BRAKE,
        wait=True,
        settle_time=DEFAULT_SETTLE_DELAY,
    ):
        self.drive_base.straight(distance, then, wait)
        if settle_time:
            sleep(settle_time)

    def smart_drive_for_distance(
        self,
        distance,
        then=Stop.BRAKE,
        k_p=2.5,
        k_i=0.01,
        k_d=0.2,
        delta_time=0.02,
    ):
        if not distance:
            return
        pid = PIDController(k_p, k_i, k_d, delta_time)
        target_heading = self.hub.imu.heading()
        self.drive_base.reset()
        direction = 1 if distance >= 0 else -1
        while abs(self.drive_base.distance()) < abs(distance):
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            correction = pid.calculate(error)
            self.drive_base.drive(direction, -correction)
            sleep(delta_time)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        else:
            self.drive_base.stop()

    def turn_in_place(
        self, degrees, then=Stop.BRAKE, wait=True
    ):
        adjusted = degrees * 1.25
        self.hub.imu.reset_heading(0)
        self.drive_base.turn(-adjusted, then, wait)
        sleep(DEFAULT_SETTLE_DELAY)

    def smart_turn_in_place(
        self,
        target_angle,
        then=Stop.BRAKE,
        k_p=3.5,
        k_i=0.02,
        k_d=0.3,
        delta_time=0.02,
        allowed_error=2.0
    ):
        pid = PIDController(k_p, k_i, k_d, delta_time)
        target_heading = self.wrap_angle(self.hub.imu.heading() + target_angle)
        self.drive_base.stop()
        while True:
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < allowed_error:
                break
            correction = pid.calculate(error)
            self.drive_base.drive(0, -correction)
            sleep(delta_time)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        else:
            self.drive_base.stop()

    def curve(self, radius, angle, then=Stop.COAST, wait=True):
        self.drive_base.curve(radius, -angle*1.25, then, wait)

    def change_drive_settings(self, reset=False, speed=None, acceleration=None, turn_rate=None, turn_acceleration=None):
        if reset:
            self.drive_profile = DRIVE_PROFILE.copy()
            print(self.drive_profile)
            self.drive_base.settings(**self.drive_profile)
            return
        if speed is not None:
            self.drive_profile["straight_speed"] = speed
        if acceleration is not None:
            self.drive_profile["straight_acceleration"] = acceleration
        if turn_rate is not None:
            self.drive_profile["turn_rate"] = turn_rate
        if turn_acceleration is not None:
            self.drive_profile["turn_acceleration"] = turn_acceleration
        self.drive_base.settings(**self.drive_profile)

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
        self.hub.light.off()
        self.hub.light.on(color)
        return color


class MissionControl:
    def __init__(self, robot:Robot):
        self.robot = robot
        self.missions = MISSION_REGISTRY
        default_slots = sorted(self.missions.keys())
        self.menu_options = [s for s in default_slots if s != "C"]
        self.stopwatch = StopWatch()
        self.last_run = None

    def build_menu(self):
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

    def execute_mission(self, selection):
        mission = self.missions.get(selection)
        self.robot.hub.display.animate(RUNNING_ANIMATION, 30)
        print("Running #{}...".format(selection))
        self.stopwatch.reset()
        self.robot.drive_for_distance(-10, settle_time=0)
        self.robot.hub.imu.reset_heading(0)
        self.robot.change_drive_settings(reset=True)
        mission(self.robot)
        elapsed = self.stopwatch.time()
        print("Done running #{} in {}ms".format(selection, elapsed))
        return selection

    def run(self):
        self.battery_status = self.robot.battery_display()
        while True:
            menu = self.build_menu()
            selection = hub_menu(*menu)
            self.last_run = self.execute_mission(selection)

def run_1(r:Robot):
    # ALIGN THE 
    # Sweep
    r.drive_for_distance(644) # Drive up to the sweep box
    r.smart_turn_in_place(-90) # Face thing
    r.drive_for_distance(20)
    r.smart_turn_in_place(-30) # Sweep left
    r.smart_turn_in_place(60) # Right
    r.smart_turn_in_place(-30) # Back to middle
    r.drive_for_distance(-100) # Drive back
    r.rotate_left_motor(90) # Arm down
    sleep(1000) # Wait for brush to stop moving
    r.drive_for_distance(90, then=Stop.COAST, speed=300) # Drive forward to align and pick up the brush
    r.rotate_left_motor(-90, speed=200, wait=False) # Pick up brush (hopefully), move arm up, yeet into the oval
    sleep(1000)
    r.drive_for_distance(-50) # Drive back a bit

    # Map
    r.smart_turn_in_place(90) # Turn to move to map
    r.drive_for_distance(60) # Go to map
    r.turn_in_place(-74) # Turn to face map
    r.rotate_left_motor(90) # Arm down
    r.drive_for_distance(185) # Push map section
    r.smart_turn_in_place(12) # Finish rotating the map section
    r.smart_turn_in_place(-12)
    r.rotate_left_motor(-90, speed=200, wait=False)
    r.smart_turn_in_place(29)
    r.drive_for_distance(80)
    r.rotate_left_motor(90)
    r.drive_for_distance(15)

    # Post map alignment
    r.drive_for_distance(-120) # Leave map
    r.rotate_left_motor(-135, wait=False) # Move long arm out the way
    r.turn_in_place(60) # Face wall
    r.drive_for_distance(500, stop=Stop.COAST) # Go to wall to align

def run_2(r:Robot):
    # Minecart
    r.drive_for_distance(-220) # Drive back from wall
    r.smart_turn_in_place(90) # Get in position for minecart
    r.drive_for_distance(170)

    # Detour for the statue
    r.rotate_right_motor(-40, then=Stop.COAST, wait=False) # Move short arm onto ground
    r.drive_for_distance(215) # Arrive at statue
    sleep(1000)
    r.rotate_right_motor(30) # Push statue up
    r.drive_for_distance(36, then=Stop.COAST, wait=False) # Drive to statue and push it up more
    r.rotate_right_motor(40, then=Stop.COAST) # Moving arm up more
    r.drive_for_distance(-280)

    # Back to the minecart
    r.rotate_right_motor(-80, then=Stop.COAST, wait=False) # Move short arm onto ground
    r.smart_turn_in_place(-90)
    r.drive_for_distance(65) # Arrive at minecart
    r.rotate_right_motor(70, speed=100) # Push up minecart
    sleep(1000) # Wait for minecart to roll down
    r.drive_for_distance(-120)
    r.rotate_right_motor(-30) # Arm down

my_robot = Robot()
run_1(my_robot)

# # Button press
# while True:
#     # Get the set of buttons currently pressed
#     pressed_buttons = my_robot.hub.buttons.pressed() 

#     # Check for a specific button being pressed
#     if Button.CENTER in pressed_buttons:
#         run_1(my_robot)