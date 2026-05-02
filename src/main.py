#!/usr/bin/env pybricks-micropython
# Default db.settings(): (straight_speed=217, straight_acceleration=816, turn_rate=160, turn_acceleration=720)
# Default motor stall_tolerances: (speed=20, time=200)

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

DRIVEBASE_WHEEL_DIAMETER = 62.4  # Medium treaded wheel diameter (mm)
DRIVEBASE_AXLE_TRACK = 130


class LebobDriveBase(DriveBase):
    def _stop_with(self, then):
        self.stop()
        if then == Stop.HOLD:
            ld.hold()
            rd.hold()
        elif then == Stop.BRAKE:
            ld.brake()
            rd.brake()

    def _wait_until_stalled(
        self, speed_tolerance: int = 0, turn_tolerance: int = 0, then=Stop.COAST
    ):
        """Wait until the robot stalls based on tolerance thresholds.

        Args:
            speed_tolerance: Max drive speed (mm/s) to consider stalled. 0 uses default stalled().
            turn_tolerance: Max turn rate (deg/s) to consider stalled. 0 uses default stalled().
            then: What to do after stalling (Stop.COAST, Stop.BRAKE, Stop.HOLD).
        """
        if speed_tolerance == 0 and turn_tolerance == 0:
            while not self.stalled():
                wait(10)
        else:
            wait(200)  # Let motors spin up before checking
            while True:
                state = self.state()
                if abs(state[1]) <= speed_tolerance and abs(state[3]) <= turn_tolerance:
                    break
                wait(10)
        self._stop_with(then)

    def straight_until_stalled(
        self, speed: int = None, tolerance: int = 0, then=Stop.COAST
    ):
        """Drive straight until the robot stalls.

        Args:
            speed: Drive speed in mm/s. Positive drives forward, negative drives backward.
                   Defaults to the configured straight_speed from settings.
            tolerance: Max speed (mm/s) to consider stalled. 0 uses default stall detection.
            then: What to do after stalling (Stop.COAST, Stop.BRAKE, Stop.HOLD).
        """
        if speed is None:
            speed = self.settings()[0]
        self.drive(speed, 0)
        self._wait_until_stalled(speed_tolerance=tolerance, then=then)

    def arc_until_stalled(self, radius: float, tolerance: int = 0, then=Stop.COAST):
        """Drive in an arc until the robot stalls. Uses the configured straight_speed and
        caps the speed to stay within the configured turn_rate, matching db.arc() behavior.

        Args:
            radius: Arc radius in mm. Positive arcs left, negative arcs right.
            tolerance: Max speed (mm/s) to consider stalled. 0 uses default stall detection.
            then: What to do after stalling (Stop.COAST, Stop.BRAKE, Stop.HOLD).
        """
        straight_speed = self.settings()[0]
        max_turn_rate = self.settings()[2]
        turn_rate = straight_speed / radius * (180 / 3.14159265)
        if abs(turn_rate) > max_turn_rate:
            straight_speed = abs(max_turn_rate * radius * 3.14159265 / 180)
            turn_rate = straight_speed / radius * (180 / 3.14159265)
        self.drive(straight_speed, turn_rate)
        self._wait_until_stalled(speed_tolerance=tolerance, then=then)

    def turn_until_stalled(
        self, turn_rate: int = None, tolerance: int = 0, then=Stop.COAST
    ):
        """Turn in place until the robot stalls. Stops when either motor stalls,
        since the other wheel may just be slipping.

        Args:
            turn_rate: Turn rate in deg/s. Positive turns left, negative turns right.
                       Defaults to the configured turn_rate from settings.
            tolerance: Max turn rate (deg/s) to consider stalled. 0 uses default stall detection.
            then: What to do after stalling (Stop.COAST, Stop.BRAKE, Stop.HOLD).
        """
        if turn_rate is None:
            turn_rate = self.settings()[2]
        self.drive(0, turn_rate)
        wait(200)  # Let motors spin up
        while True:
            if ld.stalled() or rd.stalled():
                break
            if tolerance > 0:
                state = self.state()
                if abs(state[3]) <= tolerance:
                    break
            wait(10)
        self._stop_with(then)


# Hub
hub = PrimeHub()

# Drive motors
ld = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
rd = Motor(Port.C, positive_direction=Direction.CLOCKWISE)

# FLL attachment motors
lbm = Motor(Port.F, gears=[12, 20])
rbm = Motor(Port.E, gears=[12, 20])
db = LebobDriveBase(  # DriveBase
    ld,
    rd,
    wheel_diameter=DRIVEBASE_WHEEL_DIAMETER,
    axle_track=DRIVEBASE_AXLE_TRACK,
)
db.use_gyro(True)

# Missions
MISSIONS = []


def mission(func_ptr):
    """Register a mission function in menu order."""
    MISSIONS.append(func_ptr)
    return func_ptr


def reset_robot():
    db.reset()
    hub.imu.reset_heading(0)
    lbm.reset_angle(0)
    rbm.reset_angle(0)
    db.stop()
    lbm.stop()
    rbm.stop()


@mission
def mission_1():
    """Flip boulders and heavy."""
    db.straight(280)  # Drive up to silo

    db.turn(-55)
    rbm.run_angle(75, -50, wait=False)
    db.arc(265, 90)
    db.straight(60)

    lbm.run_angle(200, 180, wait=False)
    rbm.run_until_stalled(-125, then=Stop.COAST)  # Arm down
    rbm.run_until_stalled(150, then=Stop.HOLD)  # Arm back up

    # Return
    db.drive(-1000, 0)
    wait(100)
    db.drive(-1000, -80)
    wait(2000)


@mission
def mission_2():
    """Silo."""
    db.straight(400)
    db.turn(10)
    lbm.run_target(500, 90)
    for _ in range(3):
        lbm.dc(100)
        wait(500)
        lbm.run_target(500, 135)


@mission
def mission_3():
    """Scales and raise pan."""
    # Drive to raise
    db.straight(220)
    db.turn(-45)
    db.straight(210)
    rbm.run_angle(380, -140, then=Stop.COAST, wait=False)  # Arm down onto the platform
    wait(800)
    rbm.stop()
    db.settings(straight_speed=100)
    db.straight(-70)
    rbm.run_angle(400, 150, wait=False)  # Raise
    db.straight(-26)  # Pull the platform up at the same time
    db.settings(straight_speed=400)
    db.straight(80)  # Let go of the arm and go up to the scales

    # Go to pan
    rd.run_time(
        400, 1000, then=Stop.COAST
    )  # Turn left by only moving the right wheel, and knock the scales
    db.turn(-50)
    db.straight(110)
    db.arc(
        131,
        164,
    )
    db.straight(20)
    rbm.run_angle(400, -160)  # Arm down to hit pan
    wait(200)
    rbm.run_angle(600, 120)
    db.straight(-110)  # Take pan out
    db.settings(straight_speed=1000)
    db.turn(-95)
    db.straight(60)
    db.turn_until_stalled(tolerance=20, then=Stop.HOLD)
    lbm.dc(100)
    rd.run_time(-50, 1500)
    lbm.stop()
    db.straight(-50)
    db.turn(35)
    db.arc(-650, 60)
    db.straight(600)


@mission
def mission_4():
    """Angler artifacts."""

    
@mission
def mission_5():
    """Ship."""
    db.settings(straight_speed=200)
    db.straight_until_stalled(tolerance=40)
    db.settings(straight_speed=1000)
    db.straight(-700)


@mission
def mission_6():
    """Brush and broom."""
    # Brush
    lbm.run_angle(200, 90, wait=False)
    rbm.run_angle(200, 90, wait=False)
    db.straight(650)  # Drive forward and push brush forward
    lbm.run_angle(200, 95)
    db.straight(-120)
    lbm.run_angle(200, -90)
    db.straight(105)

    # Move to map
    db.turn(45)
    db.straight(134)
    db.turn(-88)  # Face map
    db.straight(150)

    rbm.run_angle(200, -90)  # Pick up liftable map
    db.straight(-140)
    db.turn(55)
    db.straight(-750)


@mission
def mission_7():
    """Minecart."""
    db.straight(910)
    db.turn(90)  # Face minecart
    db.straight(-90, wait=False)  # Give space for arms
    rbm.run_time(-200, 800, Stop.COAST, False)  # Right arm down
    lbm.run_until_stalled(200, Stop.COAST, 50)  # Left arm down

    db.settings(straight_speed=100)
    db.straight(170)  # Drive into the minecart area
    lbm.run_angle(200, -12, wait=False)  # Pick up artefact
    rbm.dc(85)
    rbm.reset_angle(0)
    current_angle = rbm.angle()
    print(current_angle)
    while rbm.angle() < current_angle + 90:
        wait(10)
    print(rbm.angle())
    rbm.stop()
    wait(350)
    lbm.run_angle(200, -25, then=Stop.COAST, wait=False)
    db.straight(-170)  # Return

    db.settings(straight_speed=1000)
    lbm.run_angle(100, -45, wait=False)
    rbm.run_angle(200, 80, wait=False)
    db.turn(93)
    db.straight(810)


@mission
def mission_8():
    """Forum, Statue and Flags"""
    db.arc(250, 45)
    db.straight(150)
    db.settings(straight_speed=200)
    db.straight(-170)
    lbm.run_angle(300, -90)
    db.settings(straight_speed=1000)
    db.turn(50)
    db.straight(300)
    lbm.run_until_stalled(300, then=Stop.COAST, duty_limit=25)
    db.arc(-150, 50)
    db.straight(20)
    lbm.run_angle(300, -90, then=Stop.COAST)
    wait(500)
    db.turn(10)
    db.straight(150)
    db.turn(-35)
    db.straight(550)


def mission_selector():
    mission_index = 0

    while True:
        hub.display.char(str(mission_index + 1))
        pressed = hub.buttons.pressed()

        if Button.LEFT in pressed:
            mission_index = (mission_index + 1) % len(MISSIONS)
            while hub.buttons.pressed():
                wait(20)

        elif Button.RIGHT in pressed:
            mission_index = (mission_index - 1) % len(MISSIONS)
            while hub.buttons.pressed():
                wait(20)

        elif Button.CENTER in pressed:
            while hub.buttons.pressed():
                wait(20)
            hub.light.on(Color.GREEN)
            timer = StopWatch()
            try:
                timer.resume()
                MISSIONS[mission_index]()
            finally:
                print(f"Time Elapsed: {timer.time()}ms")
                reset_robot()
                hub.light.on(Color.BLUE)
            mission_index = (mission_index + 1) % len(MISSIONS)

        wait(20)


def main():
    # Battery
    voltage = hub.battery.voltage()
    percentage = max(0, min(100, (voltage - 6000) * 100 // (8400 - 6000)))
    print(f"Battery: {str(percentage)}% ({str(voltage)}mV)")
    print(f"Settings: {tuple(db.settings())}")

    # Buttons
    hub.system.set_stop_button(Button.BLUETOOTH)
    hub.display.orientation(Side.BOTTOM)
    hub.light.on(Color.RED if percentage < 80 else Color.BLUE)  # Ready
    reset_robot()
    db.settings(
        straight_speed=1000,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )

    mission_selector()  # Start mission selector


main()

