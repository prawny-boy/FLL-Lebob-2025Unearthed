#!/usr/bin/env pybricks-micropython

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import hub_menu, wait

DRIVEBASE_WHEEL_DIAMETER = 62.4  # Medium treaded wheel diameter (mm)
DRIVEBASE_AXLE_TRACK = 150

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

MISSIONS = []


def mission(func):
    """Register a mission function in menu order."""
    MISSIONS.append(func)
    return func


def reset_robot_state():
    drive_base.stop()
    left_big_motor.hold()
    right_big_motor.hold()


@mission
def mission_1():
    """Example mission. Replace this with your own code."""
    drive_base.straight(200)
    drive_base.turn(90)
    drive_base.straight(-200)


def mission_selector():
    if not MISSIONS:
        while True:
            hub.display.number(0)
            wait(200)

    while True:
        options = [str(index + 1) for index in range(len(MISSIONS))]
        selected = hub_menu(*options)
        mission_index = int(selected) - 1

        hub.display.number(mission_index + 1)
        hub.light.on(Color.GREEN)
        try:
            MISSIONS[mission_index]()
        finally:
            reset_robot_state()
            hub.light.on(Color.BLUE)
        wait(250)


def main():
    hub.system.set_stop_button(Button.BLUETOOTH)
    reset_robot_state()
    hub.light.on(Color.BLUE)
    mission_selector()


main()
