#!/usr/bin/env pybricks-micropython

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, Matrix

DRIVEBASE_WHEEL_DIAMETER = 62.4  # Medium treaded wheel diameter (mm)
DRIVEBASE_AXLE_TRACK = 130
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
db = DriveBase(
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
    #  rbm.run_time(200, 3000, then=Stop.COAST, wait=False)
    lbm.run_time(-200, 2000, then=Stop.COAST, wait=False)
    rbm.run_time(-200, 500, then=Stop.COAST, wait=False)
    db.straight(675)
    db.turn(-25)
    lbm.run_time(100, 1000, then=Stop.COAST, wait=True)
    db.straight(130)
    rbm.run_angle(400, 90, wait=True)
    db.use_gyro(False)
    db.turn(10)
    # db.turn(-5)
    db.use_gyro(True)
    db.settings(straight_speed=250)
    db.straight(-130)
    db.settings(straight_speed=500)
    db.straight(-200)


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


# lbm.dc(-100)
# wait(500) #  Slam arm down
# rbm.dc(100)
# wait(1000)


@mission
def mission_6():
    pass


def mission_selector():
    mission_index = 3

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
