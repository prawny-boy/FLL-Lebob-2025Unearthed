#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class Robot:
    def __init__(self):
        # Initialize the motors.
        left_motor = Motor(Port.B)
        right_motor = Motor(Port.C)

        line_sensor = ColorSensor(Port.S3)

        robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5 , axle_track=104)

    def drive(self, speed=100, turn_rate=0):
        self.robot.drive(speed, turn_rate)
