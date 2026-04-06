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
left_drive = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
right_drive = Motor(Port.C, positive_direction=Direction.CLOCKWISE)

# FLL attachment motors
left_big_motor = Motor(Port.F)
right_big_motor = Motor(Port.E)

# DriveBase
drive_base = DriveBase(
    left_drive,
    right_drive,
    wheel_diameter=DRIVEBASE_WHEEL_DIAMETER,
    axle_track=DRIVEBASE_AXLE_TRACK,
)

drive_base.use_gyro(True)


MISSIONS = []

# Spinning dot around the border of the 5x5 display (clockwise)
_BORDER = [
    (0,0),(0,1),(0,2),(0,3),(0,4),
    (1,4),(2,4),(3,4),(4,4),
    (4,3),(4,2),(4,1),(4,0),
    (3,0),(2,0),(1,0),
]
_RUNNING_ANIMATION = [
    [[100 if (row == r and col == c) else 0 for col in range(5)] for row in range(5)]
    for r, c in _BORDER
]


def mission(func):
    """Register a mission function in menu order."""
    MISSIONS.append(func)
    return func


def reset_robot_state():
    drive_base.stop()
    left_big_motor.stop()
    right_big_motor.stop()


@mission
def mission_1(easy_mode: bool = True, scale: float = 1):
    """Movement test."""
    if easy_mode:
        drive_base.straight(400)
        drive_base.turn(10)
        left_big_motor.run_target(500, 90)
        for _ in range(3):
            left_big_motor.dc(100)
            wait(500)
            left_big_motor.run_target(500, 135)

    else:
        drive_base.straight(100 * scale)
        drive_base.turn(113)
        drive_base.straight(-250 * scale)
        drive_base.turn(-106)
        drive_base.straight(168 * scale)
        drive_base.turn(80)
        drive_base.straight(220 * scale)
        drive_base.straight(200 * scale)
        drive_base.straight(240 * scale)
        drive_base.turn(136.2056)
        drive_base.straight(10 * scale)
        drive_base.straight(277 * scale)
        drive_base.turn(-19.5244)
        drive_base.straight(-310 * scale)
        drive_base.straight(-10 * scale)
        drive_base.turn(401.2188)
        drive_base.straight(300 * scale)
        drive_base.turn(-180)
        drive_base.straight(-375 * scale)
        drive_base.turn(48)
        drive_base.straight(250 * scale)
        drive_base.turn(-113)
        drive_base.straight(-100 * scale)


@mission
def mission_2():
    """Do brush and map."""
    drive_base.settings(straight_speed=500)
    left_big_motor.run_time(200, 1000, then=Stop.COAST, wait=False)
    #  right_big_motor.run_time(200, 2000, then=Stop.COAST, wait=False)
    drive_base.straight(690)
    drive_base.turn(-25)
    drive_base.straight(130)
    right_big_motor.run_time(-1000, 1000, wait=True)
    #  left_big_motor.run_time(-1000, 1000, wait=False)
    drive_base.straight(-130)
    drive_base.turn(35)
    drive_base.straight(-200)
    drive_base.turn(15)
    drive_base.straight(-PID)
    #  drive_base.turn(-60)
    #  drive_base.straight(-300)

@mission
def mission_3():
    """Do your minecart and artefact."""
    global hub
    hub.speaker.play_notes(["C4/4", "D4/4", "E4/4"], 500)
    drive_base.settings(straight_speed=350)
    drive_base.straight(898)
    drive_base.turn(88)  # Face minecart
    left_big_motor.run_time(200, 870, then=Stop.COAST, wait=False)  # Arms back
    right_big_motor.run_time(-200, 800, then=Stop.COAST, wait=False)
    drive_base.straight(-100)  # Give space for arms
    drive_base.settings(straight_speed=100)
    drive_base.straight(180)
    left_big_motor.run_angle(200, -10, then=Stop.HOLD)  # Pick up thing
    right_big_motor.run_angle(550, 110)  # Push up minecart track
    drive_base.straight(-50)
    drive_base.settings(straight_speed=500)
    drive_base.straight(-160)  # Return
    drive_base.turn(90)
    left_big_motor.run_time(200, -80, then=Stop.COAST, wait=False)  # Arms back
    drive_base.straight(810)

@mission
def mission_4():
    right_big_motor.dc(-100)
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
            hub.display.animate(_RUNNING_ANIMATION, interval=80)
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
