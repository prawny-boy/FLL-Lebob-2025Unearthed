from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop
from pybricks.tools import wait as sleep, Matrix, StopWatch, hub_menu
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub

DRIVEBASE_WHEEL_DIAMETER = 88 # 56 is small, 88 is big
DRIVEBASE_AXLE_TRACK = 115 # confirm this value
LOW_VOLTAGE = 7000
HIGH_VOLTAGE = 8300
MENU_OPTIONS = ["1", "2", "3", "4", "5", "6", "7", "8", "C"]
ROBOT_SETTINGS = {
    "straight_speed": 500,
    "straight_acceleration": 750,
    "turn_rate": 300,
    "turn_acceleration": 500,
}
ROBOT_MAX_TORQUE = 700
battery_status_light = Color.GREEN

class Robot:
    def __init__(self, use_gyro=False):
        """Initialises the Robot:
            Port A: Small Motor (Left Drive)
            Port B: Small Motor (Right Drive)
            Port C: Medium Motor
            Port D: Medium Motor
            Port E: Empty
            Port F: Empty
        """
        # Motors
        self.left_drive = Motor(Port.C)
        self.right_drive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
        self.right_big = Motor(Port.A)
        self.left_big = Motor(Port.B)

        # Drivebase Settings
        self.drive_base = DriveBase(self.left_drive, self.right_drive, DRIVEBASE_WHEEL_DIAMETER, DRIVEBASE_AXLE_TRACK)
        self.drive_base.settings(straight_speed=ROBOT_SETTINGS["straight_speed"], straight_acceleration=ROBOT_SETTINGS["straight_acceleration"], turn_rate=ROBOT_SETTINGS["turn_rate"], turn_acceleration=ROBOT_SETTINGS["turn_acceleration"])
        self.drive_base.settings(**ROBOT_SETTINGS)

        # Defines the hub
        self.hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
        self.hub.system.set_stop_button(Button.BLUETOOTH)
        
        # Using Gyro for movement
        self.hub.imu.reset_heading(0)
        self.drive_base.use_gyro(use_gyro=use_gyro)
        
    def rotate_right_motor(self, degrees, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.BRAKE, wait=True):
        self.right_big.run_angle(speed, degrees, then, wait)
    
    def rotate_left_motor(self, degrees, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.BRAKE, wait=True):
        self.left_big.run_angle(speed, degrees, then, wait)
    
    def rotate_right_motor_until_stalled(self, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.COAST, duty_limit=50):
        self.right_big.run_until_stalled(speed, then, duty_limit)

    def rotate_left_motor_until_stalled(self, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.COAST, duty_limit=20):
        self.left_big.run_until_stalled(speed, then, duty_limit)

    def wrap_angle(self, angle):
        # Keep angle within [-180, 180]
        return (angle + 180) % 360 - 180
    
    def drive_for_distance(self, distance, then=Stop.BRAKE, wait=True):
        self.drive_base.straight(distance, then, wait)
        sleep(250)

    def smart_drive_for_distance(self, 
                                 distance, 
                                 speed=ROBOT_SETTINGS["straight_speed"], 
                                 then=Stop.BRAKE, 
                                 k_p=2.5, 
                                 k_i=0.01, 
                                 k_d=0.2, 
                                 loop_delay_time=0.02):
        integral = 0
        previous_error = 0
        target_heading = self.hub.imu.heading()
        self.drive_base.reset()
        while abs(self.drive_base.distance()) < abs(distance):
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            proportional = error
            integral += error * loop_delay_time
            derivative = (error - previous_error) / loop_delay_time
            correction = k_p * proportional + k_i * integral + k_d * derivative
            # Drive forward with turn correction
            self.drive_base.drive(speed, -correction)
            previous_error = error
            sleep(loop_delay_time)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        elif then == Stop.COAST:
            self.drive_base.stop()
    
    def turn_in_place(self, degrees, then="brake"):
        degrees *= 1.25
        self.hub.imu.reset_heading(0)
        self.drive_base.turn(degrees*-1, Stop.COAST, True)
        if then == "stop":
            self.drive_base.stop()
        else:
            self.drive_base.brake()
        sleep(250)

    def smart_turn_in_place(self, 
                            target_angle, 
                            then=Stop.BRAKE,
                            k_p=3.5, k_i=0.02, k_d=0.3,
                            loop_delay_time=0.02):
        integral = 0
        previous_error = 0
        target_heading = self.wrap_angle(self.hub.imu.heading() + target_angle)
        self.drive_base.stop()
        while True:
            current_heading = self.hub.imu.heading()
            error = self.wrap_angle(target_heading - current_heading)
            if abs(error) < 2.0:
                break
            proportional = error
            integral += error * loop_delay_time
            derivative = (error - previous_error) / loop_delay_time
            correction = k_p * proportional + k_i * integral + k_d * derivative
            # Turn with correction
            self.drive_base.drive(0, -correction)
            previous_error = error
            print(f"Heading: {current_heading:.2f} Error: {error:.2f} Corr: {correction:.2f}")
            sleep(loop_delay_time)
        if then == Stop.BRAKE:
            self.drive_base.brake()
        elif then == Stop.COAST:
            self.drive_base.stop()
    
    def curve(self, radius, angle, then=Stop.COAST, wait=True):
        self.drive_base.curve(radius, angle, then, wait)

    def status_light(self, color):
        self.hub.light.off()
        self.hub.light.on(color)
    
    def battery_display(self):
        # display battery of hub
        v = self.hub.battery.voltage()
        v_pct = rescale(v, LOW_VOLTAGE, HIGH_VOLTAGE, 1, 100)
        print(f"Battery %: {round(v_pct, 1)}, Voltage: {v}")
        if v_pct < 40:
            print("EMERGENCY: BATTERY LOW!")
            battery_status_light = Color.RED
        if v_pct < 70:
            print("Battery is below 70% Please charge!")
            battery_status_light = Color.YELLOW
        else:
            battery_status_light = Color.GREEN
        self.status_light(battery_status_light)
        return battery_status_light
    
    def clean_motors(self):
        self.left_drive.run_angle(999, 1000, wait=False)
        self.right_drive.run_angle(999, 1000, wait=False)
        self.left_big.run_angle(999, 1000, wait=False)
        self.right_big.run_angle(999, 1000)

running_animation = [
    Matrix([
        [0, 0, 100, 100, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 100, 100, 0, 0]
    ]), Matrix([
        [100, 0, 0, 100, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 100, 0, 0, 100]
    ]), Matrix([
        [100, 100, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 100, 100]
    ]), Matrix([
        [100, 100, 100, 0, 0],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [0, 0, 100, 100, 100]
    ]), Matrix([
        [100, 100, 100, 100, 0],
        [100, 0, 0, 0, 0],
        [100, 0, 0, 0, 100],
        [0, 0, 0, 0, 100],
        [0, 100, 100, 100, 100]
    ]), Matrix([
        [100, 100, 100, 100, 100],
        [100, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 100],
        [100, 100, 100, 100, 100]
    ]), Matrix([
        [100, 100, 100, 100, 100],
        [0, 0, 0, 0, 100],
        [0, 0, 0, 0, 0],
        [100, 0, 0, 0, 0],
        [100, 100, 100, 100, 100]
    ]), Matrix([
        [0, 100, 100, 100, 100],
        [0, 0, 0, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 0, 0, 0, 0],
        [100, 100, 100, 100, 0]
    ])
]

def mission_function_one(r:Robot):
    """
    Start Location Description:
    What It Does:
    """
    r.rotate_left_motor_until_stalled(-100)
    r.rotate_right_motor_until_stalled(-100)
    r.drive_for_distance(755)
    r.turn_in_place(-90)
    r.drive_for_distance(65)
    r.rotate_left_motor(98)
    r.turn_in_place(35)
    r.turn_in_place(-70)
    r.turn_in_place(35)
    r.drive_for_distance(-95)
    r.turn_in_place(5)
    r.drive_for_distance(110)
    r.rotate_left_motor(-115)
    r.drive_for_distance(-100)
    r.turn_in_place(150)
    r.drive_for_distance(155)
    r.rotate_right_motor_until_stalled(50)
    r.turn_in_place(-55)
    r.rotate_right_motor(-55)
    sleep(2000)
    r.rotate_right_motor_until_stalled(100)
    r.drive_for_distance(-100)
    r.turn_in_place(50)
    r.rotate_right_motor_until_stalled(-100)
    r.drive_for_distance(-200)
    r.turn_in_place(-45)
    r.drive_for_distance(-700)

def mission_function_two(r:Robot):
    r.drive_for_distance(200)
    r.turn_in_place(90)
    r.drive_for_distance(400)
    r.rotate_right_motor_until_stalled(200)
    r.drive_for_distance(-50)
    r.rotate_right_motor(-120)
    r.drive_for_distance(100)
    r.drive_for_distance(-50)
    r.turn_in_place(-45)
    r.drive_for_distance(100)
    r.turn_in_place(45)
    r.drive_for_distance(800)
    
def mission_function_three(r:Robot):
    r.drive_for_distance(730)
    r.turn_in_place(30)
    r.drive_for_distance(30)
    r.turn_in_place(-110)
    # r.turn_in_place(135)
    # r.drive_for_distance(-70)
    # r.turn_in_place(-70)
    # r.drive_for_distance(130)
    # r.turn_in_place(-180)
    r.drive_for_distance(150)
    r.turn_in_place(-200)

def mission_function_four(r:Robot):
    r.drive_for_distance(200)
    r.turn_in_place(-45)
    r.drive_for_distance(300, wait=False)
    r.rotate_left_motor_until_stalled(-100)
    r.rotate_right_motor_until_stalled(-100)
    sleep(500)
    r.rotate_left_motor(120, wait=False)
    r.rotate_right_motor(120, speed=500)
    r.rotate_right_motor(-120)
    r.drive_for_distance(-700)

def mission_function_five(r:Robot):
    r.drive_for_distance(100)
    r.turn_in_place(-30)
    r.rotate_left_motor_until_stalled(100)
    r.drive_for_distance(900)
    r.rotate_left_motor(-120, speed=500, wait=False)
    sleep(200)
    r.drive_for_distance(50)

def mission_function_six(r:Robot):
    r.smart_turn_in_place(90)
    r.smart_turn_in_place(90)
    r.smart_turn_in_place(90)
    r.smart_turn_in_place(90)
    r.drive_for_distance(1000)

def mission_function_seven(r:Robot):
    pass

def mission_function_eight(r:Robot):
    pass

# Utility functions
def rescale(value, in_min, in_max, out_min, out_max):
    neg = value / abs(value) # will either be 1 or -1
    value = abs(value)
    if value < in_min: value = in_min
    if value > in_max: value = in_max
    retvalue = (value - in_min) * (out_max / (in_max - in_min))
    if retvalue > out_max: retvalue = out_max
    if retvalue < out_min: retvalue = out_min
    return retvalue * neg

def stopwatch(command="start"):
    global my_stopwatch
    if command == "start":
        my_stopwatch.pause()
        my_stopwatch.reset()
        my_stopwatch.resume()
    else:
        return my_stopwatch.time()

def run_mission(r:Robot, selected):
    global my_stopwatch
    # run current selection
    r.status_light(Color.YELLOW)
    r.hub.display.animate(running_animation, 30)
    print(f"Running #{selected}...")
    stopwatch("start")
    r.drive_for_distance(-10)
    r.hub.imu.reset_heading(0)
    if selected == "1":
        mission_function_one(r)
    elif selected == "2":
        mission_function_two(r)
    elif selected == "3":
        mission_function_three(r)
    elif selected == "4":
        mission_function_four(r)
    elif selected == "5":
        mission_function_five(r)
    elif selected == "6":
        mission_function_six(r)
    elif selected == "7":
        mission_function_seven(r)
    elif selected == '8':
        mission_function_eight(r)
    print(f"Done running #{selected} in {stopwatch('stop')}ms")
    r.status_light(battery_status_light)
    return selected

# create objects
my_robot = Robot(use_gyro=False)
my_stopwatch = StopWatch()

# display battery
battery_status_light = my_robot.battery_display()

# run menu
last_run = "C"
while True:
    current_menu = []
    for i in range(len(MENU_OPTIONS)):
        current_menu.append(MENU_OPTIONS[(i+MENU_OPTIONS.index(last_run)+1) % len(MENU_OPTIONS)])
    selected = hub_menu(*current_menu)
    if selected != "C":
        last_run = run_mission(my_robot, selected)
    else:
        my_robot.clean_motors()