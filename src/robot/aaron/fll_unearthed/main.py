#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Axis, Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile

class PIDController:
    """I know the solultion to all your problems."""

    def __init__(
        self,
        k_p,
        k_i,
        k_d,
        loop_delay_time=0.02,
        integral_limit=None,
        output_limit=None,
    ):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.loop_delay_time = loop_delay_time
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0
        self.previous_error = 0

    def calculate(self, error):
        self.integral += error * self.loop_delay_time
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        derivative = (error - self.previous_error) / self.loop_delay_time
        output = self.k_p * error + self.k_i * self.integral + self.k_d * derivative
        if self.output_limit is not None:
            output = max(-self.output_limit, min(output, self.output_limit))
        self.previous_error = error
        return output

class Robot:
    def __init__(self):
        # Initialize the motors.
        self.left_drive_motor = Motor(Port.A)
        self.left_medium_motor = Motor(Port.B)
        self.right_medium_motor = Motor(Port.C)
        self.right_drive_motor = Motor(Port.D)

        self.drive_base = DriveBase(self.left_drive_motor, self.right_drive_motor, wheel_diameter=91.5 , axle_track=233)
        
        self.default_settings = {
            "straight_speed": 500,
            "straight_acceleration": 750,
            "turn_rate": 300,
            "turn_acceleration": 500,
        }
        self.use_gyro = True
        self.reset_settings()

        self.hub = EV3Brick(top_side=Axis.Z, front_side=Axis.X)
        self.hub.imu.reset_heading(0)

    def reset_settings(self):
        self.drive_base.settings(**self.default_settings)
        self.drive_base.use_gyro(use_gyro=self.use_gyro)

    def wrap_angle(self, angle):
        return (angle + 180) % 360 - 180
    
    def stop_drivebase(self, then=Stop.BRAKE):
        if then == Stop.COAST:
            self.drive_base.stop()
        else:
            self.drive_base.brake()

    def drive_with_turn_rate(self, speed=100, turn_rate=0):
        self.robot.drive(speed, turn_rate)

    def drive_for_distance(self, distance, then=Stop.BRAKE, wait=True):
        self.drive_base.straight(distance, then, wait)

    def turn_in_place(self, degrees, then=Stop.BRAKE, wait=True):
        self.drive_base.turn(degrees, then, wait)

    def smart_turn_in_place(
        self,
        target_angle,
        then=Stop.HOLD,
        k_p=3.5,
        k_i=0.02,
        k_d=0.3,
        loop_delay_time=0.02,
    ):
        pid = PIDController(k_p, k_i, k_d, loop_delay_time, output_limit=400)
        target_heading = self.wrap_angle(self.hub.imu.heading() + target_angle)
        self.drive_base.stop()

        while True:
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < 2.0:
                break
            correction = pid.calculate(error)
            self.drive_base.drive(0, -correction)

            wait(loop_delay_time)
        self.stop_drivebase(then)

    def rotate_attachment(self, side, degrees, speed=300, then=Stop.BRAKE, wait=True):
        if side == "left":
            motor = self.left_medium_motor
        elif side == "right":
            motor = self.right_medium_motor

        motor.run_angle(speed, degrees, then, wait)

def ev3():
    r = Robot()
    r.drive_for_distance(1000)

ev3()
