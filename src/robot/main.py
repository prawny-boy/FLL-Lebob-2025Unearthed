#!/usr/bin/env pybricks-micropython

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

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


MISSIONS = []


def mission(func):
    """Register a mission function in menu order."""
    MISSIONS.append(func)
    return func


def reset_robot_state():
    db.stop()
    lbm.stop()
    rbm.stop()


@mission
def mission_1(easy_mode: bool = True, scale: float = 1):
    """Movement test."""
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
    db.straight(680)
    db.turn(-25)
    db.straight(130)
    rbm.run_angle(400, 90, wait=True)
    db.turn(-5)
    db.straight(-140)
    db.turn(35)
    db.straight(-120)
    lbm.run_angle(150, 70)
    db.straight(-90)
    db.turn(10)
    db.straight(-PID) # The solution to all our [problems] is PID!!!!!!!!!!!!!!!!!!!!!!!

@mission
def mission_3():
    """Do your minecart and artefact."""
    global hub
    hub.speaker.play_notes(["C4/4", "D4/4", "E4/4"], 500)
    db.settings(straight_speed=350)
    db.straight(898)
    db.turn(88)  # Face minecart
    lbm.run_time(200, 870, then=Stop.COAST, wait=False)  # Arms back
    rbm.run_time(-200, 800, then=Stop.COAST, wait=False)
    db.straight(-100)  # Give space for arms
    db.settings(straight_speed=100)
    db.straight(180)
    lbm.run_angle(200, -10, then=Stop.HOLD)  # Pick up thing
    rbm.run_angle(550, 110)  # Push up minecart track
    db.straight(-50)
    db.settings(straight_speed=500)
    db.straight(-160)  # Return
    db.turn(90)
    lbm.run_time(200, -80, then=Stop.COAST, wait=False)  # Arms back
    db.straight(810)

@mission
def mission_4():
    rbm.dc(-100)
    wait(1000)

def mission_selector():
    if not MISSIONS:
        while True:
            hub.display.char("0")
            wait(200)

    mission_index = 0

    while True:
        hub.display.char(str(mission_index + 1))
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
    hub.system.set_stop_button(Button.BLUETOOTH)
    hub.display.orientation(Side.BOTTOM)
    reset_robot_state()
    voltage = hub.battery.voltage()
    percentage = max(0, min(100, (voltage - 6000) * 100 // (8400 - 6000)))
    print("Battery: " + str(percentage) + "% (" + str(voltage) + " mV)")
    hub.light.on(Color.BLUE)
    mission_selector()


main()
