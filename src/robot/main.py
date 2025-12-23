"""PrimeHub entry point for the FLL Unearthed 2025 robot."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import Matrix, StopWatch, hub_menu
from pybricks.tools import wait as sleep

# bee movie

DRIVEBASE_WHEEL_DIAMETER = 88  # 56 is small, 88 is big
DRIVEBASE_AXLE_TRACK = 115
LOW_VOLTAGE = 7200
HIGH_VOLTAGE = 8400
DRIVE_PROFILE = {
    "straight_speed": 500,
    "straight_acceleration": 1000,
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


class LQRController:
    """Small LQR helper reused by the smart drive and turn helpers."""

    def __init__(
        self,
        delta_time=0.02,
        q_angle=6.0,
        q_rate=1.5,
        r=0.25,
        output_limit=720,
        iterations=80,
    ):
        self.delta_time = delta_time
        self.output_limit = output_limit
        self.k_angle, self.k_rate = self._solve_heading_gain(
            delta_time, q_angle, q_rate, r, iterations
        )
        self.previous_error = 0.0

    def _solve_heading_gain(self, delta_time, q_angle, q_rate, r, iterations):
        # Discrete-time Riccati solution for a double-integrator yaw model:
        # x = [heading error, yaw rate], u = steering command.
        P00, P01, P11 = q_angle, 0.0, q_rate
        dt = delta_time
        for _ in range(iterations):
            s = r + (dt * dt) * P11
            inv_s = 1.0 / s
            k0 = dt * P01 * inv_s
            k1 = dt * (P01 * dt + P11) * inv_s

            atpa00 = P00
            atpa01 = P00 * dt + P01
            atpa11 = dt * dt * P00 + 2 * dt * P01 + P11

            atpb0 = dt * P01
            atpb1 = dt * dt * P01 + P11 * dt
            btpa0 = dt * P01
            btpa1 = dt * (P01 * dt + P11)

            term00 = atpb0 * btpa0 * inv_s
            term01 = atpb0 * btpa1 * inv_s
            term11 = atpb1 * btpa1 * inv_s

            P00 = atpa00 - term00 + q_angle
            P01 = atpa01 - term01
            P11 = atpa11 - term11 + q_rate
        return k0, k1

    def reset(self):
        self.previous_error = 0.0

    def calculate(self, error):
        error_rate = (error - self.previous_error) / self.delta_time
        control = self.k_angle * error + self.k_rate * error_rate
        if self.output_limit is not None:
            limit = self.output_limit
            if control > limit:
                control = limit
            elif control < -limit:
                control = -limit
        self.previous_error = error
        return control


class Robot:
    """Wrapper around the PrimeHub, drive base, and attachments."""

    def __init__(self, use_gyro=False, drive_profile=None):
        profile = DRIVE_PROFILE.copy()
        if drive_profile:
            profile.update(drive_profile)
        self.drive_profile = profile

        self.left_drive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
        self.right_drive = Motor(Port.C) #, Direction.COUNTERCLOCKWISE)
        self.right_big = Motor(Port.E)
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

    def zero_heading(self, heading=0):
        """Reset IMU heading to a known reference."""
        self.hub.imu.reset_heading(heading)
        return heading

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
        speed=300,
        delta_time=0.02,
        q_angle=6.0,
        q_rate=1.5,
        r=0.25,
        turn_limit=720,
    ):
        if not distance:
            return
        controller = LQRController(
            delta_time=delta_time,
            q_angle=q_angle,
            q_rate=q_rate,
            r=r,
            output_limit=turn_limit,
        )
        target_heading = self.hub.imu.heading()
        self.drive_base.reset()
        direction = 1 if distance >= 0 else -1
        while abs(self.drive_base.distance()) < abs(distance):
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            correction = controller.calculate(error)
            self.drive_base.drive(direction*speed, -correction)
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
        delta_time=0.02,
        q_angle=6.0,
        q_rate=1.5,
        r=0.2,
        allowed_error=2.0,
        turn_limit=720,
    ):
        controller = LQRController(
            delta_time=delta_time,
            q_angle=q_angle,
            q_rate=q_rate,
            r=r,
            output_limit=turn_limit,
        )
        target_heading = self.wrap_angle(self.hub.imu.heading() + target_angle)
        self.drive_base.stop()
        while True:
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < allowed_error:
                break
            correction = controller.calculate(error)
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
        self.robot.zero_heading()
        self.robot.hub.display.animate(RUNNING_ANIMATION, 30)
        print("Running #{}...".format(selection))
        self.stopwatch.reset()
        self.robot.drive_for_distance(-10, settle_time=0)
        self.robot.zero_heading()
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


def mission(slot):
    """Decorator used to register mission handlers."""

    def decorator(func):
        MISSION_REGISTRY[slot] = func
        return func

    return decorator


@mission("1")
def mission_function_one(robot:Robot):
    robot.change_drive_settings(speed=1000)
    robot.drive_for_distance(740) # Go up to sweep
    robot.change_drive_settings(reset=True)
    robot.turn_in_place(-90) # Turn to face the sweep
    robot.drive_for_distance(75) # Go forward a lot to align
    robot.drive_for_distance(-30) # Go back to give space for the arm
    robot.rotate_left_motor_until_stalled(100) # Align the arm to the frame
    robot.rotate_left_motor(-65) # Move the arm up to the right height to pick up
    robot.turn_in_place(35) # Sweep left
    robot.change_drive_settings(turn_rate=100)
    robot.turn_in_place(-60) # Sweep right
    robot.change_drive_settings(reset=True)
    robot.turn_in_place(28) # Return to middle
    robot.drive_for_distance(-95) # Go back
    robot.turn_in_place(10)
    sleep(400) # Wait for brush to stop swaying.
    robot.drive_for_distance(100)
    robot.rotate_left_motor(-115, speed=100) # Pick up brush
    # robot.drive_for_distance(-110)
    robot.turn_in_place(30) # Turn to map reveal
    robot.drive_for_distance(-30)
    robot.rotate_right_motor_until_stalled(100, then=Stop.HOLD)
    robot.change_drive_settings(speed=100)
    robot.drive_for_distance(75) # Push map
    robot.change_drive_settings(reset=True)
    robot.rotate_right_motor(-110)
    robot.drive_for_distance(100)
    robot.change_drive_settings(speed=1000, turn_rate=1000)
    robot.drive_for_distance(-200)
    robot.turn_in_place(60) # Turn to face the other start area
    robot.drive_for_distance(-800) # Drive to other start area
    robot.change_drive_settings(reset=True)

@mission("2")
def mission_function_two(robot:Robot):
    robot.change_drive_settings(speed=1000)
    robot.drive_for_distance(1000)
    robot.drive_for_distance(-145)
    robot.hub.imu.reset_heading(0)
    robot.change_drive_settings(reset=True)
    robot.curve(100, 92)
    robot.drive_for_distance(-100)
    robot.rotate_left_motor(-100, wait=False)
    robot.rotate_right_motor_until_stalled(500)
    robot.change_drive_settings(speed=100)
    robot.drive_for_distance(110)
    robot.change_drive_settings(speed=1000)
    robot.rotate_right_motor(-70, speed=100, wait=False)
    robot.rotate_left_motor(40)
    sleep(1000)
    robot.rotate_right_motor(60)
    robot.turn_in_place(10)
    robot.drive_for_distance(-180)
    robot.rotate_left_motor(80, wait=False)
    robot.rotate_right_motor(-90, wait=False)
    robot.smart_turn_in_place(90)
    robot.drive_for_distance(850)


@mission("3")
def mission_function_three(robot:Robot):
    robot.rotate_right_motor_until_stalled(-100) # Reset arm
    robot.drive_for_distance(210) # Drive forward
    robot.smart_turn_in_place(90) # Turn to face shipwreck
    robot.change_drive_settings(speed=500)
    robot.drive_for_distance(600) # Drive to shipwreck
    robot.drive_for_distance(-40) # Move backwards make space
    robot.smart_turn_in_place(-(robot.hub.imu.heading()-90))
    robot.rotate_right_motor_until_stalled(50, duty_limit=75) # Move arm onto ground to pull the lever
    robot.rotate_right_motor(-45)
    robot.drive_for_distance(-200) # Move backwards to pull the lever
    robot.change_drive_settings(speed=1000, acceleration=1000)
    robot.drive_for_distance(35)
    robot.rotate_right_motor(-80, wait=False) # Move arm back up so it's no in the way
    robot.turn_in_place(-45) # Start driving to the other start area
    robot.curve(250, 70)
    robot.drive_for_distance(1000) # Drive to other start area


@mission("4")
def mission_function_four(robot:Robot):
    robot.rotate_left_motor_until_stalled(100) # Reset arm
    robot.drive_for_distance(30) # Move forward to give space for turning
    robot.smart_turn_in_place(-15) # Turn to face the mission
    robot.drive_for_distance(680) # Drive to mission (flipping the platform)
    robot.change_drive_settings(turn_rate=50)
    robot.turn_in_place(45)
    robot.change_drive_settings(reset=True)
    robot.rotate_right_motor_until_stalled(200)
    robot.smart_turn_in_place(45)
    robot.rotate_right_motor(-100)
    robot.smart_turn_in_place(-38)
    robot.drive_for_distance(70) # Move into the boulders
    robot.change_drive_settings(turn_rate=50)
    robot.turn_in_place(-75) # Rotate to flip the platform and push the boulders
    robot.change_drive_settings(reset=True)
    robot.drive_for_distance(-203) # Go back to give space to return
    robot.turn_in_place(-43) # Face the raising platform
    robot.drive_for_distance(160) # Move to raising platform
    robot.turn_in_place(15)
    #robot.drive_for_distance(50)
    robot.rotate_left_motor_until_stalled(-200, then=Stop.HOLD) # Move arm down, move down the bucket
    robot.rotate_left_motor(30)
    robot.change_drive_settings(speed=300)
    robot.drive_for_distance(-650, wait=False) # Move back to flip the platform
    robot.change_drive_settings(speed=500)
    sleep(300)
    robot.rotate_left_motor(45) # Return to starting area
    robot.drive_for_distance(75)
    robot.rotate_left_motor(100)
    robot.drive_for_distance(-700)


@mission("5")
def mission_function_five(robot:Robot):
    # mission 4, will be combining 4 & 5
    robot.drive_for_distance(30) # Forward to give space
    robot.smart_turn_in_place(-15)
    robot.hub.imu.reset_heading(-15)
    robot.drive_for_distance(320)
    robot.curve(55, -120) # raise the goods
    robot.drive_for_distance(230)
    robot.smart_turn_in_place((robot.hub.imu.heading()-182))
    robot.hub.imu.reset_heading(-90)
    robot.drive_for_distance(210)
    robot.smart_turn_in_place(90)
    robot.change_drive_settings(speed=200)
    robot.drive_for_distance(100)
    robot.drive_for_distance(-100)
    robot.change_drive_settings(speed=1000, acceleration=1000)
    robot.turn_in_place(-90)
    robot.drive_for_distance(60) # Drive up to the statue
    robot.change_drive_settings(reset=True)
    robot.smart_turn_in_place(45) # Face statue MANY INCONSISTENCIES WITH THIS ONE
    robot.rotate_left_motor_until_stalled(-500) # Move arm to ground
    robot.drive_for_distance(300) # Drive up to the statue so the arm is under it
    robot.rotate_left_motor(0, then=Stop.COAST)
    robot.rotate_left_motor(120, speed=1000) # Lift statue up
    robot.drive_for_distance(-100)


@mission("T")
def test_mission_function(robot:Robot):
    pass

@mission("6")
def mission_function_six(robot:Robot):
    robot.drive_for_distance(200)
    robot.rotate_right_motor(360*5, speed=1000)
    robot.rotate_right_motor(1000, speed=-2000)
    robot.drive_for_distance(-100)
    robot.turn_in_place(180)
    robot.drive_for_distance(200)

def rescale(value, in_min, in_max, out_min, out_max):
    if value < in_min:
        value = in_min
    elif value > in_max:
        value = in_max
    scale = (value - in_min) / (in_max - in_min)
    return out_min + scale * (out_max - out_min)


def main():
    MissionControl(Robot(use_gyro=False)).run()


main()
