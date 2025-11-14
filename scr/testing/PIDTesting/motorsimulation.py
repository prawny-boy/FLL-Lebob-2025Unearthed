import matplotlib.pyplot as plt
from pidcontroller import PIDController

class VirtualMotor:
    """Simple motor simulation with inertia and response to duty cycle."""
    def __init__(self):
        self.angle = 0
        self.speed = 0

    def update(self, duty, dt):
        max_accel = 1000  # deg/s^2 for max duty
        accel = max_accel * (duty / 100)
        self.speed += accel * dt
        self.angle += self.speed * dt
        return self.angle, self.speed

motor = VirtualMotor()
pid = PIDController(Kp=0.5, Ki=0.1, Kd=0.05)

target_angle = 720 # degrees
max_speed = 1000 # deg/s
dt = 0.01 # seconds
time_total = 2 # seconds

times, angles, speeds, duties = [], [], [], []

t = 0
while t < time_total:
    duty = pid.to_angle_with_speed(target_angle, motor.angle, motor.speed, max_speed, dt)
    angle, speed = motor.update(duty, dt)

    times.append(t)
    angles.append(angle)
    speeds.append(speed)
    duties.append(duty)

    t += dt

plt.figure(figsize=(10, 6))

plt.subplot(3,1,1)
plt.plot(times, angles)
plt.ylabel('Angle (deg)')
plt.axhline(target_angle, color='r', linestyle='--', label='Target')
plt.legend()

plt.subplot(3,1,2)
plt.plot(times, speeds)
plt.ylabel('Speed (deg/s)')
plt.axhline(max_speed, color='r', linestyle='--', label='Max speed')
plt.legend()

plt.subplot(3,1,3)
plt.plot(times, duties)
plt.ylabel('Duty cycle')
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
