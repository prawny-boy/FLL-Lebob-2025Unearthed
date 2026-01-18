"""PrimeHub entry point for the FLL Unearthed 2025 robot."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import Matrix, StopWatch, hub_menu
from pybricks.tools import wait as sleep


DRIVEBASE_WHEEL_DIAMETER = 62.4  # Medium treaded wheel diameter (mm)
DRIVEBASE_AXLE_TRACK = 150
# Optional open-loop turn scaling (leave at 1.0 when geometry is correct).
TURN_CORRECTION = 1.0
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
ATTACHMENT_JOG_SPEED = 400
ATTACHMENT_POLL_DELAY_MS = 50
RUNNING_ANIMATION = tuple(
    Matrix(frame)
    for frame in (
        [
            [0, 0, 100, 100, 100],
            [100, 100, 0, 0, 100],
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
        delta_time=0.02,
        integral_limit=None,
        output_limit=None,
    ):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.delta_time = delta_time
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0
        self.previous_error = 0

    def calculate(self, error):
        self.integral += error * self.delta_time
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        derivative = (error - self.previous_error) / self.delta_time
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

        self.left_drive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
        self.right_drive = Motor(Port.C)
        self.right_big = Motor(Port.E)
        self.left_big = Motor(Port.F)

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
        smart=True,
        speed=None,
        k_p=1.6,
        k_i=0.01,
        k_d=0.2,
        delta_time=0.02,
        heading_tolerance=1.0,
        turn_limit=None,
        distance_k_p=1.2,
        distance_k_i=0.0,
        distance_k_d=0.05,
        distance_tolerance=5,
        minimum_speed=40,
        slow_down_distance=150,
    ):
        if not distance:
            return
        if not smart:
            self.drive_base.straight(distance, then, wait)
            if settle_time:
                sleep(settle_time)
            return

        loop_delay_ms = max(1, int(delta_time * 1000))
        resolved_speed = speed if speed is not None else self.drive_profile["straight_speed"]
        resolved_turn_limit = (
            turn_limit if turn_limit is not None else self.drive_profile.get("turn_rate", 300)
        )
        heading_pid = PIDController(k_p, k_i, k_d, delta_time, output_limit=resolved_turn_limit)
        distance_pid = PIDController(
            distance_k_p,
            distance_k_i,
            distance_k_d,
            delta_time,
            output_limit=abs(resolved_speed),
        )
        target_heading = self.hub.imu.heading()
        self.drive_base.reset()
        direction = 1 if distance >= 0 else -1 # Goon Checkpoint

        while True:
            traveled = self.drive_base.distance()
            distance_error = distance - traveled
            if abs(distance_error) <= distance_tolerance:
                break
            current_heading = self.hub.imu.heading()
            heading_error = self.wrap_angle(target_heading - current_heading)
            turn_correction = heading_pid.calculate(heading_error)
            if abs(distance_error) > slow_down_distance:
                linear_speed = direction * abs(resolved_speed)
            else:
                linear_speed = distance_pid.calculate(distance_error)
                if abs(linear_speed) < minimum_speed:
                    linear_speed = direction * minimum_speed
                linear_speed = max(-abs(resolved_speed), min(linear_speed, abs(resolved_speed)))
            self.drive_base.drive(linear_speed, turn_correction)
            sleep(loop_delay_ms)
        # Hold heading briefly while stopping so we do not finish with a swerve.
        self.drive_base.stop()
        heading_pid.reset()
        heading_pid.previous_error = self.wrap_angle(target_heading - self.hub.imu.heading())
        for _ in range(5):
            current_heading = self.hub.imu.heading()
            heading_error = self.wrap_angle(target_heading - current_heading)
            if abs(heading_error) <= heading_tolerance:
                break
            turn_correction = heading_pid.calculate(heading_error)
            self.drive_base.drive(0, turn_correction)
            sleep(loop_delay_ms)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        else:
            self.drive_base.stop()
        if settle_time:
            sleep(settle_time)

    def turn_in_place(
        self,
        degrees,
        then=Stop.BRAKE,
        wait=True,
        smart=True,
        k_p=1.6,
        k_i=0.0,
        k_d=0.2,
        delta_time=0.02,
        allowed_error=0.15,
        turn_limit=None,
        max_iterations=200,
    ):
        if not smart:
            self.hub.imu.reset_heading(0)
            self.drive_base.turn(-degrees * TURN_CORRECTION, then, wait)
            sleep(DEFAULT_SETTLE_DELAY)
            return

        loop_delay_ms = max(1, int(delta_time * 1000))
        base_turn_limit = turn_limit if turn_limit is not None else self.drive_profile.get("turn_rate", 300)
        # Default to a brisk turn rate; larger requests get a higher floor.
        resolved_turn_limit = max(base_turn_limit, 500 if abs(-degrees) >= 45 else 360)
        minimum_turn_rate = max(resolved_turn_limit * 0.06, 10)
        fine_tune_turn_rate = max(resolved_turn_limit * 0.03, 6)
        pid = PIDController(k_p, k_i, k_d, delta_time, output_limit=resolved_turn_limit)
        target_heading = self.wrap_angle(self.hub.imu.heading() + degrees)
        self.drive_base.stop()
        prev_error = self.wrap_angle(target_heading - self.hub.imu.heading())
        consecutive_hits = 0
        for _ in range(max_iterations):
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < allowed_error:
                consecutive_hits += 1
                if consecutive_hits >= 2:
                    break
            else:
                consecutive_hits = 0
            # If we overshoot and are within a tight band, stop immediately.
            if error * prev_error < 0 and abs(error) < 2 * allowed_error:
                break
            correction = pid.calculate(error)
            # Stay fast for most of the move; ramp down only near the target to keep accuracy.
            error_mag = abs(error)
            if error_mag < 1.0:
                effective_limit = max(resolved_turn_limit * 0.12, 28)
                min_rate = fine_tune_turn_rate
            elif error_mag < 5:
                effective_limit = max(resolved_turn_limit * 0.28, 110)
                min_rate = minimum_turn_rate
            elif error_mag < 12:
                effective_limit = max(resolved_turn_limit * 0.5, 200)
                min_rate = minimum_turn_rate
            else:
                effective_limit = resolved_turn_limit
                min_rate = minimum_turn_rate
            correction = max(-effective_limit, min(correction, effective_limit))
            if error_mag < 12 and abs(correction) < min_rate:
                correction = min_rate if error > 0 else -min_rate
            self.drive_base.drive(0, correction)
            prev_error = error
            sleep(loop_delay_ms)
        # Tiny settle to pull into the tighter window without creeping.
        pid.reset()
        hold_limit = min(resolved_turn_limit, 24)
        settle_min_rate = max(hold_limit * 0.35, 6)
        for _ in range(2):
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) <= allowed_error / 2:
                break
            correction = pid.calculate(error)
            correction = max(-hold_limit, min(correction, hold_limit))
            if abs(correction) < settle_min_rate and abs(error) > allowed_error / 3:
                correction = settle_min_rate if error > 0 else -settle_min_rate
            self.drive_base.drive(0, correction)
            sleep(loop_delay_ms)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        else:
            self.drive_base.stop()
        sleep(DEFAULT_SETTLE_DELAY)

    def curve(self, radius, angle, then=Stop.COAST, wait=True):
        self.drive_base.curve(radius, -angle * TURN_CORRECTION, then, wait)

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


def mission(slot):
    """Decorator used to register mission handlers."""

    def decorator(func):
        MISSION_REGISTRY[slot] = func
        return func

    return decorator


@mission("1")
def mission_function_one(robot:Robot):
    robot.change_drive_settings(speed=1000)
    robot.drive_for_distance(535)
    robot.change_drive_settings(reset=True)
    robot.rotate_left_motor(-145)
    robot.drive_for_distance(-50)
    robot.rotate_left_motor(145)
    robot.drive_for_distance(200)
    robot.turn_in_place(-45)
    robot.drive_for_distance(200)
    robot.rotate_right_motor(360)
    robot.drive_for_distance(-200)
    robot.turn_in_place(55)
    robot.drive_for_distance(-1100)

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
    robot.turn_in_place(90, smart=True)
    robot.drive_for_distance(850)


@mission("3")
def mission_function_three(robot:Robot):
    robot.rotate_right_motor_until_stalled(-100) # Reset arm
    robot.drive_for_distance(210) # Drive forward
    robot.turn_in_place(90, smart=True) # Turn to face shipwreck
    robot.change_drive_settings(speed=500)
    robot.drive_for_distance(600) # Drive to shipwreck
    robot.drive_for_distance(-40) # Move backwards make space
    robot.turn_in_place(-(robot.hub.imu.heading()-90), smart=True)
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
    robot.turn_in_place(-15, smart=True) # Turn to face the mission
    robot.drive_for_distance(680) # Drive to mission (flipping the platform)
    robot.change_drive_settings(turn_rate=50)
    robot.turn_in_place(45)
    robot.change_drive_settings(reset=True)
    robot.rotate_right_motor_until_stalled(200)
    robot.turn_in_place(45, smart=True)
    robot.rotate_right_motor(-100)
    robot.turn_in_place(-38, smart=True)
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
    robot.turn_in_place(-15, smart=True)
    robot.hub.imu.reset_heading(-15)
    robot.drive_for_distance(320)
    robot.curve(55, -120) # raise the goods
    robot.drive_for_distance(230)
    robot.turn_in_place((robot.hub.imu.heading()-182), smart=True)
    robot.hub.imu.reset_heading(-90)
    robot.drive_for_distance(210)
    robot.turn_in_place(90, smart=True)
    robot.change_drive_settings(speed=200)
    robot.drive_for_distance(100)
    robot.drive_for_distance(-100)
    robot.change_drive_settings(speed=1000, acceleration=1000)
    robot.turn_in_place(-90)
    robot.drive_for_distance(60) # Drive up to the statue
    robot.change_drive_settings(reset=True)
    robot.turn_in_place(45, smart=True) # Face statue MANY INCONSISTENCIES WITH THIS ONE
    robot.rotate_left_motor_until_stalled(-500) # Move arm to ground
    robot.drive_for_distance(300) # Drive up to the statue so the arm is under it
    robot.rotate_left_motor(0, then=Stop.COAST)
    robot.rotate_left_motor(120, speed=1000) # Lift statue up
    robot.drive_for_distance(-100)


@mission("T")
def test_mission_function(robot:Robot):
    robot.drive_for_distance(300, smart=True)
    for _ in range(3):
        robot.drive_for_distance(300, smart=True)
        robot.turn_in_place(90, smart=True)
    robot.drive_for_distance(-300, smart=True)

@mission("6")
def mission_function_six(robot:Robot):
    robot.drive_for_distance(200)
    robot.rotate_right_motor(360*5, speed=1000)
    robot.rotate_right_motor(1000, speed=-2000)
    robot.drive_for_distance(-100)
    robot.turn_in_place(180)
    robot.drive_for_distance(200)

@mission("7")
def mission_function_seven(robot:Robot):
    robot.rotate_right_motor(-1000, speed=1000)
    robot.rotate_right_motor(1000, speed=1000)


@mission("M")
def mission_function_manual_attachment(robot:Robot):
    def wait_for_button_release():
        while robot.hub.buttons.pressed():
            sleep(ATTACHMENT_POLL_DELAY_MS)

    def manual_motor_control(motor: Motor):
        wait_for_button_release()
        while True:
            pressed = set(robot.hub.buttons.pressed())
            if Button.LEFT in pressed and Button.RIGHT not in pressed:
                motor.run(-ATTACHMENT_JOG_SPEED)
            elif Button.RIGHT in pressed and Button.LEFT not in pressed:
                motor.run(ATTACHMENT_JOG_SPEED)
            else:
                motor.stop()
            if Button.CENTER in pressed:
                motor.stop(Stop.HOLD)
                wait_for_button_release()
                break
            sleep(ATTACHMENT_POLL_DELAY_MS)

    selection = hub_menu("L", "R")
    if selection == "L":
        manual_motor_control(robot.left_big)
    elif selection == "R":
        manual_motor_control(robot.right_big)

def rescale(value, in_min, in_max, out_min, out_max):
    if value < in_min:
        value = in_min
    elif value > in_max:
        value = in_max
    scale = (value - in_min) / (in_max - in_min)
    return out_min + scale * (out_max - out_min)


def main():
    MissionControl(Robot(use_gyro=False)).run()
# I know the solution to all your problems. Add more PID.

if __name__ == "__main__":
    main()
