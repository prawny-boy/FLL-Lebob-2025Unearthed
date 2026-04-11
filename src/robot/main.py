#!/usr/bin/env pybricks-micropython

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, Matrix

DRIVEBASE_WHEEL_DIAMETER = 62.4  # Medium treaded wheel diameter (mm)
DRIVEBASE_AXLE_TRACK = 130


class LebobDriveBase(DriveBase):
    def straight_until_stalled(self, speed: int):
        """Drive straight until the robot stalls.

        Args:
            speed: Drive speed in mm/s. Positive drives forward, negative drives backward.
        """
        self.drive(speed, 0)
        while not self.stalled():
            wait(10)
        self.stop()

    def arc_until_stalled(self, radius: float):
        """Drive in an arc until the robot stalls. Uses the configured straight_speed and
        caps the speed to stay within the configured turn_rate, matching db.arc() behavior.

        Args:
            radius: Arc radius in mm. Positive arcs left, negative arcs right.
        """
        straight_speed = self.settings()[0]
        max_turn_rate = self.settings()[2]
        turn_rate = straight_speed / radius * (180 / 3.14159265)
        if abs(turn_rate) > max_turn_rate:
            straight_speed = abs(max_turn_rate * radius * 3.14159265 / 180)
            turn_rate = straight_speed / radius * (180 / 3.14159265)
        self.drive(straight_speed, turn_rate)
        while not self.stalled():
            wait(10)
        self.stop()
PID = 550

# Hub
hub = PrimeHub()

# Drive motors
ld = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
rd = Motor(Port.C, positive_direction=Direction.CLOCKWISE)

# FLL attachment motors
lbm = Motor(Port.F)
rbm = Motor(Port.E)

# DriveBase
db = LebobDriveBase(
    ld,
    rd,
    wheel_diameter=DRIVEBASE_WHEEL_DIAMETER,
    axle_track=DRIVEBASE_AXLE_TRACK,
)

db.use_gyro(True)

# Numbers
NUMBERS_MATRIX = {
    1: Matrix(
        [
            [0, 1, 1, 1, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 1, 1, 1, 0],
        ]
    )
    * 100,
    3: Matrix(
        [
            [1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1],
        ]
    )
    * 100,
    4: Matrix(
        [
            [1, 1, 0, 1, 1],
            [1, 1, 0, 1, 1],
            [0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1],
            [1, 1, 0, 1, 1],
        ]
    )
    * 100,
    6: Matrix(
        [
            [0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0],
        ]
    )
    * 100,
    7: Matrix(
        [
            [1, 0, 1, 1, 1],
            [1, 0, 0, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 1, 1, 0, 1],
        ]
    )
    * 100,
    9: Matrix(
        [
            [1, 0, 1, 0, 1],
            [0, 0, 0, 0, 0],
            [1, 0, 1, 0, 1],
            [0, 0, 0, 0, 0],
            [1, 0, 1, 0, 1],
        ]
    )
    * 100,
    10: Matrix(
        [
            [0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 1, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0],
        ]
    )
    * 100,
}

USE_DEFAULT_NUMBERS = [2, 5, 8]

# Missions
MISSIONS = []


def mission(func):
    """Register a mission function in menu order."""
    MISSIONS.append(func)
    return func



def reset_headings():
    db.reset()
    hub.imu.reset_heading(0)
    lbm.reset_angle(0)
    rbm.reset_angle(0)


def reset_robot_state():
    db.stop()
    lbm.stop()
    rbm.stop()


@mission
def mission_1(easy_mode: bool = True, scale: float = 1):
    """Movement test."""
    reset_headings()
    if easy_mode:
        db.straight(400)
        db.turn(10)
        lbm.run_target(500, 90)
        for _ in range(3):
            lbm.dc(100)
            wait(500)
            lbm.run_target(500, 135)

    else:
        db.straight(100 * scale)
        db.turn(113)
        db.straight(-250 * scale)
        db.turn(-106)
        db.straight(168 * scale)
        db.turn(80)
        db.straight(220 * scale)
        db.straight(200 * scale)
        db.straight(240 * scale)
        db.turn(136.2056)
        db.straight(10 * scale)
        db.straight(277 * scale)
        db.turn(-19.5244)
        db.straight(-310 * scale)
        db.straight(-10 * scale)
        db.turn(401.2188)
        db.straight(300 * scale)
        db.turn(-180)
        db.straight(-375 * scale)
        db.turn(48)
        db.straight(250 * scale)
        db.turn(-113)
        db.straight(-100 * scale)


@mission
def mission_2():
    """Do brush and map."""
    db.settings(straight_speed=500)
    # Brush
    db.straight(650)  # Drive forward and push brush forward
    lbm.run_angle(200, -95)
    db.straight(-95)
    lbm.run_angle(200, 90)
    db.straight(70)

    # Move to map
    db.turn(45)
    db.straight(134)
    db.turn(-88)  # Face map
    db.straight(150)

    rbm.run_angle(200, 90)  # Pick up liftable map
    db.straight(-140)
    db.turn(55)
    db.straight(-750)


@mission
def mission_3():
    """Do your minecart and artefact."""
    db.settings(straight_speed=350)
    db.straight(905)
    db.turn(90.1)  # Face minecart
    rbm.run_time(-200, 800, then=Stop.COAST, wait=False)
    lbm.stop()
    db.straight(-90)  # Give space for arms
    lbm.run_time(200, 870, then=Stop.COAST)  # Left arm down
    db.settings(straight_speed=100)
    db.straight(170)  # Drive into the minecart area
    lbm.run_angle(200, -12)  # Pick up artefact
    rbm.dc(100)  # Push up minecart track
    wait(500)
    lbm.run_angle(150, -25, then=Stop.COAST, wait=False)
    db.straight(-50)
    rbm.stop()
    db.settings(straight_speed=500)
    db.straight(-160)  # Return
    db.turn(90)
    lbm.run_time(200, -80, then=Stop.COAST, wait=False)  # Arms back
    db.straight(810)


@mission
def mission_4():
    """Silo, flip, boulders, heavy."""
    db.settings(straight_speed=400)
    db.straight(310)  # Drive up to silo
    lbm.hold()  # Hold arm up to prevent wobbling

    # smash_silo_times = 3
    # for _ in range(smash_silo_times):
    #    rbm.dc(-40) # Smash siloun_angle(400, 90)
    #    wait(500)
    #    rbm.stop()
    #    rbm.dc(90)
    #    wait(500)  # Arm back up
    #    rbm.hold()
    #    wait(800) # Stop wobbling

    db.turn(-55.1195)
    db.arc(300, 85)  # THIS ARC IS MAGIC, DO NOT TOUCH
    db.turn(-10)

    db.settings(straight_speed=150, turn_rate=130)
    # lbm.run_angle(400, 100, wait=False)
    lbm.run_until_stalled(400)
    rbm.run_angle(600, -90, then=Stop.COAST)  # Arm to hit heavy
    wait(400)
    db.turn(40)  # Turn and push heavy off

    # Arm back up
    rbm.dc(85)
    wait(410)
    rbm.stop()

    # Return
    # TODO: Make into an arc
    db.settings(straight_speed=400, turn_rate=90, turn_acceleration=180)
    db.straight(-80)
    db.turn(-80)
    rbm.run_angle(
        300, -25, wait=False
    )  # Raise arm so it fits TODO: Check if this works
    db.straight(-700)


@mission
def mission_5():  # ship
    reset_headings()
    rbm.hold()
    db.settings(straight_speed=500)
    db.straight(450)
    db.settings(straight_speed=250)
    db.straight(200)


@mission
def mission_6():
    """Scales, raise, pan, bucket, go to other side"""
    reset_headings()
    db.settings(straight_speed=400)

    # Drive to raise
    db.straight(320)
    db.turn(-45)
    db.straight(110)
    rbm.run_angle(380, -140, then=Stop.COAST, wait=False)  # Arm down onto the platform
    wait(800)
    rbm.stop()
    db.settings(straight_speed=100)
    db.straight(-70)
    rbm.run_angle(380, 150, wait=False)  # Raise
    db.straight(-23)  # Pull the platform up at the same time
    db.settings(straight_speed=400)
    db.straight(70)  # Let go of the arm and go up to the scales

    # Go to pan
    rd.run_time(
        400, 1000, then=Stop.COAST
    )  # Turn left by only moving the right wheel, and knock the scales
    db.turn(-50)
    db.straight(80)
    db.arc_until_stalled(
        110.74476231356391916348639285701748365912793783065083274823641938508136497356380461034953749264
    )  # DO NOT CHANGE THESE VALUES, THEY NEED TO BE THIS PRECISE OR THE ROBOT WILL NOT BE ABLE TO HIT THE PAN, IT WILL BE KILOMETERS OFF AND CAUSE A DISRUPTANCE IN THE GRAVITATION FIELD OF THE SOLAR SYSTEM AND WILL RETURN TO THE ROBOT IN THE YEAR 2032 AND CAUSE IT TO EXPLODE, DO NOT CHANGE THIS VALUE, I REPEAT, DO NOT CHANGE THIS VALUE. AFTER THE GENERAL PRECAUTIONS TAKEN BY THE SPACE ENGINEERING TEAM, THE ROBOT SUCCESSFULLY HITS THE PAN AND REMOVES A DISTURBANCE IN THE MAGNETIC FIELD OF THE EARTH AFTER IT HAS FLIPPED THE NORTH AND SOUTH POLE WITH AN ANGLE OF 0.34 RADIANS, CAUSING THE ROBOT TO RETURN TO THE FUTURE AFTER CREATING A 5 DIMENSIONAL WORMHOLE WITH A 5TH HIGGS BOSON-NEUTRINO DISTUBANCE ELECTROMAGNETIC FIELD IN THE YEAR 5070, BUT THIS IT GET REDIRECTED CAUSING A WAVE FUNCTION COLLAPSE, RESULTING IN A REDUCTION OF THE FINAL INTERGALACTIC VECTOR STATE OF TIME, EVOLVING TO THE ONLY POSSIBLE EIGENVALUE. IF WE FOLLOW THE ANTITAU GENERATED BY THE ELECTRONS RELEASED FROM THE ELECTROMAGNETIC DESTABILISATION OF THE ATMOSPHERE, THE NEUTRINOS ARISE IN A HIGHER ENERGY LEVEL BUT HAVE HALF INTEGER SPIN. BUT SINCE THE TAU LEPTONS RELEASE LESS BRAKING RADIATION AS THE ELECTRONS, A DISCREPANCY IS FORMED BETWEEN THE SPEED OF THE WAVES RADIATING OUT OF THE TAU NEUTRINO DECAY OF THE HADRONICALLY INDUCED MECHANICAL FIELD. THE DISCREPANCY RESULTS IN A FINAL COLLAPSE OF THE WAVE FUNCTION SINCE SUPERPOSITIONS CAN BE ELIMINATED FROM THE DISTANCE BETWEEEN THE TAU ELECTRON AND NEUTRINOS, WHICH REPRESENTS THE ROBOT EXPLODING THROUGH THE SHEARING OF THE MAGNETIC FIELD TEARING APART ITS METALLIC PARAMETRICALLY UNIVERSALLY DEFINED TAU-LEPTON FORMING FROM THE TEARING OF VARIOUS LIGHT STATES. THE ENERGY ENERGY REMAINS THE SAME ACCORDING TO THE FIRST LAW OF THERMODYNAMICS, MEANING THE SHEARING WILL ALSO RESULT IN AN IMMENSE COMPRESSION FORCE, AS THE TEMPERATURE INCREASES FROM THE INCREASED MEAN VELOCITY, WE CAN USE THE LIMIT TO CALCULATE THE FUTURE COLLAPSED TEMPERATURE. THIS CREATES ACCESSIBLE MICROSTATES, RESULTING IN THE OPENING OF THE SYSTEM AND DISTANCING FROM THERMODYNAMIC EQUILIBRIUM AND WHILE ENTROPY DECREASES, IT EVENTUALLY RESULTS IN LESS DECAY UNTIL THE RESIDUAL ENTROPY CONSTANT IS REACHED. AFTER THE GAS HAS REACHED A LOWER ENTROPY, THIS CAN ALLOW US TO DRAW THE CONCLUSION THAT THIS WILL CREATING A NEW STAR IN THE PROCESS FROM THE HIGH LEVELS OF COMPRESSION AND LOW POSSIBLE ENTROPY STATES FROM THE COOLING TEMPERATURES. THIS STAR IS NAMED "LEBOB" IN HONOR OF THE ROBOT THAT CREATED IT, AND IT BECOMES A SYMBOL OF HUMAN INGENUITY AND THE POWER OF TECHNOLOGY. THIS MISSION IS REMEMBERED AS ONE OF THE GREATEST ACHIEVEMENTS IN HUMAN HISTORY, AND IT INSPIRES GENERATIONS TO COME TO REACH FOR THE STARS AND CREATE A BETTER FUTURE FOR ALL OF HUMANITY. THIS MISSION IS A TESTAMENT TO THE POWER OF SCIENCE AND THE HUMAN SPIRIT, AND IT SHOWS THAT WITH DETERMINATION, CREATIVITY, AND A WILLINGNESS TO TAKE RISKS, WE CAN ACHIEVE THE IMPOSSIBLE AND CREATE A BETTER WORLD FOR ALL OF US.
    # db.turn(5)
    
    rbm.run_angle(800, -150)  # Arm down to hit pan
    rbm.run_angle(200, 120)
    db.straight(-85)  # Take pan out
    db.turn(-87)
    db.straight(100)
    lbm.dc(100)
    wait(2500)


@mission
def mission_7():
    pass


def mission_selector():
    mission_index = 5

    if not MISSIONS:
        raise ValueError("MISSIONS is empty.")

    while True:
        if (mission_index + 1) in USE_DEFAULT_NUMBERS:
            hub.display.char(str(mission_index + 1))
        else:
            hub.display.icon(NUMBERS_MATRIX[mission_index + 1])
        pressed = hub.buttons.pressed()

        if Button.LEFT in pressed:
            mission_index = (mission_index + 1) % len(MISSIONS)
            while hub.buttons.pressed():
                wait(20)
            wait(120)
            continue

        if Button.RIGHT in pressed:
            mission_index = (mission_index - 1) % len(MISSIONS)
            while hub.buttons.pressed():
                wait(20)

            wait(120)
            continue

        if Button.CENTER in pressed:
            while hub.buttons.pressed():
                wait(20)
            hub.light.on(Color.GREEN)
            try:
                MISSIONS[mission_index]()
            finally:
                reset_robot_state()
                hub.light.on(Color.BLUE)
            mission_index = (mission_index + 1) % len(MISSIONS)
            wait(250)

        wait(20)


def main():
    # Buttons
    hub.system.set_stop_button(Button.BLUETOOTH)
    hub.display.orientation(Side.BOTTOM)
    reset_robot_state()  # Stop motors
    hub.light.on(Color.BLUE)  # Ready

    # Battery
    voltage = hub.battery.voltage()
    percentage = max(0, min(100, (voltage - 6000) * 100 // (8400 - 6000)))
    print("Battery: " + str(percentage) + "% (" + str(voltage) + " mV)")
    print(f"Settings: {tuple(db.settings())}")

    # Start mission selector
    mission_selector()


main()
