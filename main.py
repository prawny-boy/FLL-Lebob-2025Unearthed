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
    "turn_rate": 750,
    "turn_acceleration": 3000
}
ROBOT_MAX_TORQUE = 1000
battery_status_light = Color.GREEN

class Robot:
    def __init__(self):
        """Initialises the Robot:
            Port A: Empty
            Port B: Empty
            Port C: Empty
            Port D: Empty
            Port E: Empty
            Port F: Empty
        """
        # MOTORS: Left () Right () Right Big () Left Big ()
        self.left_drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.right_drive = Motor(Port.B)
        self.right_big = Motor(Port.C)
        self.left_big = Motor(Port.D)

        # Possible Sensors on robot???

        # Defines the drivebase
        self.drive_base = DriveBase(self.left_drive, self.right_drive, DRIVEBASE_WHEEL_DIAMETER, DRIVEBASE_AXLE_TRACK)
        self.drive_base.use_gyro(True)
        self.drive_base.settings(**ROBOT_SETTINGS)
        self.left_drive.control.limits(ROBOT_SETTINGS["straight_speed"], ROBOT_SETTINGS["straight_acceleration"], ROBOT_MAX_TORQUE)
        self.right_drive.control.limits(ROBOT_SETTINGS["straight_speed"], ROBOT_SETTINGS["straight_acceleration"], ROBOT_MAX_TORQUE)

        # Defines the hub
        self.hub = PrimeHub(front_side=-Axis.Y)
        self.hub.system.set_stop_button(Button.BLUETOOTH)
        
    def move_right_motor_in_degrees(self, degrees, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.BRAKE, wait=True):
        self.right_big.run_angle(speed, degrees, then, wait)
    
    def move_left_motor_in_degrees(self, degrees, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.BRAKE, wait=True):
        self.left_big.run_angle(speed, degrees, then, wait)
    
    def move_right_motor_until_stalled(self, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.COAST, duty_limit=50):
        self.right_big.run_until_stalled(speed, then, duty_limit)

    def move_left_motor_until_stalled(self, speed=ROBOT_SETTINGS["turn_rate"], then=Stop.COAST, duty_limit=20):
        self.left_big.run_until_stalled(speed, then, duty_limit)
    
    def drive_for_distance(self, distance, then=Stop.BRAKE, wait=True):
        self.drive_base.straight(distance, then, wait)
        sleep(250)
    
    def turn_in_place(self, degrees, then=Stop.BRAKE, wait=True):
        global selected
        self.drive_base.turn(degrees, then, wait)
        sleep(250)
    
    def curve(self, radius, angle, then=Stop.BRAKE, wait=True):
        self.drive_base.curve(radius, angle, then, wait)

    def status_light(self, color):
        self.hub.light.off()
        self.hub.light.on(color)
    
    def battery_display(self):
        # display battery of hub
        v = self.hub.battery.voltage()
        v_pct = rescale(v, LOW_VOLTAGE, HIGH_VOLTAGE, 1, 100)
        print(f"Battery %: {round(v_pct, 1)}, Voltage: {v}")
        if v_pct < 70:
            if v_pct < 40:
                print("EMERGENCY: BATTERY LOW!")
                battery_status_light = Color.RED
            else:
                print("Battery is below 70% Please charge!")
                battery_status_light = Color.YELLOW
            self.status_light(battery_status_light)
        else:
            self.status_light(Color.GREEN)
    
    def clean_motors(self):
        self.left_drive.run_angle(999, 1000, wait=False)
        self.right_drive.run_angle(999, 1000, wait=False)
        self.left_big.run_angle(999, 1000, wait=False)
        self.right_big.run_angle(999, 1000)

class Animations:
    running = [
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

class Run:
    def one(r:Robot):
        """
        Start Location Desicription:
        What It Does:
        """
        pass
 
    def two(r:Robot):
        pass
        
    def three(r:Robot):
        pass

    def four(r:Robot):
        pass

    def five(r:Robot):
        pass

    def six(r:Robot):
        pass

    def seven(r:Robot):
        pass

    def eight(r:Robot):
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

def run_mission(r:Robot, selected):
    # run current selection
    r.status_light(Color.YELLOW)
    r.hub.display.animate(Animations.running, 30)
    print(f"Running #{selected}...")
    if selected == "1":
        Run.one(r)
    elif selected == "2":
        Run.two(r)
    elif selected == "3":
        Run.three(r)
    elif selected == "4":
        Run.four(r)
    elif selected == "5":
        Run.five(r)
    elif selected == "6":
        Run.six(r)
    elif selected == "7":
        Run.seven(r)
    elif selected == '8':
        Run.eight(r)
    print(f"Done running #{selected}.")
    r.status_light(battery_status_light)
    return selected

# create robot
my_robot = Robot()

# display battery
my_robot.battery_display()

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