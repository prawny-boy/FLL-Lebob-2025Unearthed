"""PID motor simulation playground."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pidcontroller import PIDController


@dataclass
class VirtualMotor:
    max_accel: float = 1000.0  # deg/s^2
    damping: float = 0.15      # proportional speed loss

    angle: float = 0.0
    speed: float = 0.0

    def update(self, duty: float, dt: float) -> Tuple[float, float]:
        duty = max(-100.0, min(100.0, duty))
        accel = (duty / 100.0) * self.max_accel - self.damping * self.speed
        self.speed += accel * dt
        self.angle += self.speed * dt
        return self.angle, self.speed


def simulate(
    pid: PIDController,
    motor: VirtualMotor,
    target_angle: float,
    max_speed: float,
    dt: float,
    total_time: float,
) -> Tuple[List[float], List[float], List[float], List[float]]:
    times: List[float] = []
    angles: List[float] = []
    speeds: List[float] = []
    duties: List[float] = []
    t = 0.0
    while t <= total_time:
        duty = pid.to_angle_with_speed(target_angle, motor.angle, motor.speed, max_speed, dt)
        angle, speed = motor.update(duty, dt)
        times.append(t)
        angles.append(angle)
        speeds.append(speed)
        duties.append(duty)
        t += dt
    return times, angles, speeds, duties


def plot_results(
    times: List[float],
    angles: List[float],
    speeds: List[float],
    duties: List[float],
    target_angle: float,
    max_speed: float,
    output: Path,
    show: bool,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(times, angles, label="angle")
    axes[0].axhline(target_angle, color="r", linestyle="--", label="target")
    axes[0].set_ylabel("Angle (deg)")
    axes[0].legend()

    axes[1].plot(times, speeds, label="speed")
    axes[1].axhline(max_speed, color="r", linestyle="--", label="limit")
    axes[1].axhline(-max_speed, color="r", linestyle="--")
    axes[1].set_ylabel("Speed (deg/s)")
    axes[1].legend()

    axes[2].plot(times, duties, label="duty")
    axes[2].set_ylabel("Duty cycle")
    axes[2].set_xlabel("Time (s)")

    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output)
    if show:
        plt.show()
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulate PID tuning against a virtual motor")
    parser.add_argument("target_angle", type=float, help="Angle command in degrees")
    parser.add_argument("--kp", type=float, default=0.5)
    parser.add_argument("--ki", type=float, default=0.1)
    parser.add_argument("--kd", type=float, default=0.05)
    parser.add_argument("--max-speed", type=float, default=1000.0, help="Speed cap for PID inner loop")
    parser.add_argument("--dt", type=float, default=0.01, help="Simulation time-step in seconds")
    parser.add_argument("--time", type=float, default=2.0, help="Total simulated time in seconds")
    parser.add_argument("--output", type=Path, default=Path("motor_simulation.png"), help="PNG path for graphs")
    parser.add_argument("--show", action="store_true", help="Display the plot after saving")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    controller = PIDController(kp=args.kp, ki=args.ki, kd=args.kd)
    motor = VirtualMotor()
    times, angles, speeds, duties = simulate(
        controller,
        motor,
        target_angle=args.target_angle,
        max_speed=args.max_speed,
        dt=args.dt,
        total_time=args.time,
    )
    plot_results(times, angles, speeds, duties, args.target_angle, args.max_speed, args.output, args.show)
    print(f"Saved simulation plot to {args.output}")


if __name__ == "__main__":
    main()
