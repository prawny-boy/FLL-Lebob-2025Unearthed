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
    # Stop everything first so reset_angle/reset don't throw EPERM.
    db.stop()
    lbm.stop()
    rbm.stop()
    wait(50)

    db.reset()
    hub.imu.reset_heading(0)
    lbm.reset_angle(0)
    rbm.reset_angle(0)



@mission
def mission_1():
    """Flip boulders and heavy."""
    print("Mission 1: Flip boulders and heavy")
    db.straight(310)  # Drive up to silo

    db.turn(-55)
    db.arc(295, 100) # Go to flip boulders
    db.straight(10)
    
    rbm.run_angle(200, -210)
    wait(200)
    rbm.run_angle(100, 290, wait=False)
    wait(1000)
    lbm.run_angle(300, -360)

    # Return
    db.straight(-150)
    db.turn(-45)
    db.drive(-1000, -15)
    wait(2000)


@mission
def mission_2():
    """Silo."""
    rbm.run_angle(300, -45, wait=False)
    db.straight(350)
    for _ in range(4):
        rbm.dc(-90)
        wait(250)
        rbm.stop()
        wait(100)
        rbm.dc(90)
        wait(250)
    db.straight(-350)


@mission
def mission_3():
    """Scales and raise pan."""
    # Drive to raise
    lbm.run_angle(200, 120, wait=False)
    db.straight(250)
    db.turn(-43)
    db.straight(210)
    lbm.run_angle(200, -55)
    rbm.run_angle(200, 90) 
    lbm.run_angle(100, 110)
    db.settings(straight_speed=500)
    db.straight(-50)
    db.settings(straight_speed=1000)
    rbm.run_angle(50, -60, wait=False)
    db.straight(-175)
    db.straight(140)
    rbm.run_angle(300, -60, wait=False)
    db.turn(-55)
    lbm.run_angle(200, -175, wait=False)
    wait(800)
    db.straight(800, wait=False)
    wait(750)
    lbm.run_angle(50, 45)
    db.turn(-10)
    db.straight(800)

    
@mission
def mission_4():
    """Minecart."""
    db.straight_until_stalled(1000)
    hub.imu.reset_heading(0)
    rd.run(-200)
    while hub.imu.heading() < 85:
        wait(10)
    rd.stop()
    db.reset()
    db.use_gyro(True)
    rbm.run_time(-200, 800, Stop.COAST, False)  # Right arm down
    lbm.run_until_stalled(200, Stop.BRAKE, 50)  # Left arm down
    lbm.run_angle(200, -2, then=Stop.HOLD)  # Left arm back
    db.settings(straight_speed=100)
    db.straight(125)  # Drive into the minecart area
    lbm.run_angle(150, -39, wait=False)  # Pick up artefact
    rbm.run_angle(100, 125)
    lbm.run_angle(20, -15, then=Stop.COAST, wait=False)
    db.straight(-175)  # Return

    db.settings(straight_speed=1000)
    lbm.run_angle(100, -45, wait=False)
    rbm.run_angle(200, 80, wait=False)
    db.turn(93)
    db.straight(810)


@mission
def mission_5():
    """Map and Brush."""
    # Brush
    # lbm.run_angle(200, 120, wait=False)
    # rbm.run_angle(200, -90, wait=False)
    rbm.run_until_stalled(-500, duty_limit=50)
    lbm.run_until_stalled(-500, duty_limit=50)
    db.settings(straight_speed=250)
    db.straight(650)  # Drive forward and push brush forward
    db.straight(-170)
    lbm.run_until_stalled(500, duty_limit=50)
    db.settings(straight_speed=100)
    db.straight(70)
    db.settings(straight_speed=1000)
    lbm.run_until_stalled(-500, duty_limit=50)

    # Move to map
    db.turn(45)
    db.straight(210)
    db.turn(-88)  # Face map
    db.settings(straight_speed=100)
    db.straight(200)
    db.settings(straight_speed=1000)

    rbm.run_until_stalled(300)  # Pick up liftable map
    db.straight(-200)
    db.turn(55)
    db.straight(-750)


@mission
def mission_6():
    """Ship and Angler Artifacts."""
    db.settings(straight_speed=300)
    db.straight(490)
    db.settings(straight_speed=1000)
    db.straight(-50)
    rbm.run_until_stalled(100)
    db.straight(30)
    lbm.dc(100)
    wait(2000)
    lbm.stop()
    db.straight(-30, wait=False)
    rbm.run_angle(300, -90)
    db.settings(straight_speed=300)
    db.straight(-750)
    db.settings(straight_acceleration=1000)


@mission
def mission_7():
    """Forum, Statue, Flags, Opp's Minecart."""
    db.curve(478, 43)  # Drive to area
    db.straight(50)  # Go into statue
    rbm.run_until_stalled(-150, duty_limit=50)  # Reset arm
    rbm.run_angle(110, 80, wait=False)  # Lift statue slow
    db.settings(straight_speed=300)
    db.straight(65)  # Push statue
    db.settings(straight_speed=1000)
    # rbm.run_until_stalled(300, duty_limit=40)
    rbm.run_angle(200, 100, wait=False)  # Lift statue fast
    db.straight(-200)  # Leave area
    db.turn(45)  # Go to Opp minecart
    rbm.run_until_stalled(100, duty_limit=50)
    db.straight(510)
    db.turn(-55)
    db.straight(470, wait=False)
    rbm.run_angle(200, -52)  # Right arm down for minecart
    wait(1000)
    db.straight(115)  # Hook into minecart
    rbm.run_angle(300, 40)  # Pick up minecart
    lbm.run_time(500, 750, then=Stop.COAST)  # Put down flag
    wait(500)
    lbm.run_angle(300, -120, wait=False)  # Arm out of the way
    db.straight(-450)
    db.reset()
    ld.run(-500)
    while hub.imu.heading() > -75:
        wait(10)
    ld.stop()
    db.straight(50)
    rbm.run_angle(300, -65, wait=False)  # Put minecart down
    db.straight(-150)
    wait(500)
    rbm.run_angle(300, 60)
    db.straight(100)
    db.turn(-40)
    db.straight(-1200)


STORAGE_OFFSET = 0


def load_saved_mission_index():
    try:
        data = hub.system.storage(STORAGE_OFFSET, read=1)
        if len(data) == 1:
            return data[0] % len(MISSIONS)
    except Exception:
        pass
    return 0


def save_mission_index(index):
    try:
        hub.system.storage(STORAGE_OFFSET, write=bytes([index]))
    except Exception:
        pass


def mission_selector():
    mission_index = load_saved_mission_index()

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
            next_index = (mission_index + 1) % len(MISSIONS)
            save_mission_index(next_index)
            hub.light.on(Color.GREEN)
            timer = StopWatch()
            try:
                timer.resume()
                MISSIONS[mission_index]()
            finally:
                print(f"Time Elapsed: {timer.time()}ms")
                reset_robot()
                hub.light.on(Color.BLUE)
            mission_index = next_index

        wait(20)


def main():
    # Battery
    voltage = hub.battery.voltage()
    percentage = max(0, min(100, (voltage - 6000) * 100 // (8400 - 6000)))
    settings = db.settings()
    print(f"Battery: {str(percentage)}% ({str(voltage)}mV)")
    print(f"Settings: {tuple(settings) if settings else ()}")

    # Buttons
    hub.system.set_stop_button(Button.BLUETOOTH)
    hub.display.orientation(Side.BOTTOM)
    hub.light.on(Color.RED if percentage < 75 else Color.BLUE)  # Ready
    reset_robot()
    db.settings(
        straight_speed=1000,
        straight_acceleration=1000,
        turn_rate=400,
        turn_acceleration=500
    )

    mission_selector()  # Start mission selector


main()

