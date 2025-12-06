"""Standalone PID utilities for testing motor control strategies."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class PIDController:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    output_limits: Tuple[float, float] = (-100.0, 100.0)
    integral_limit: float | None = None

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0

    def _clamp(self, value: float, limits: Tuple[float, float]) -> float:
        low, high = limits
        return max(low, min(high, value))

    def _update(self, error: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = self._clamp(self.integral, (-self.integral_limit, self.integral_limit))
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return self._clamp(output, self.output_limits)

    def to_speed(self, target_speed: float, measured_speed: float, dt: float) -> float:
        """Return duty cycle to reach target speed (deg/s)."""
        error = target_speed - measured_speed
        return self._update(error, dt)

    def to_angle(self, target_angle: float, measured_angle: float, dt: float) -> float:
        """Return duty cycle to drive toward desired angle (degrees)."""
        error = target_angle - measured_angle
        return self._update(error, dt)

    def to_angle_with_speed(
        self,
        target_angle: float,
        measured_angle: float,
        measured_speed: float,
        max_speed: float,
        dt: float,
    ) -> float:
        """Nested control: angle error converted to speed target and fed into the speed loop."""
        angle_error = target_angle - measured_angle
        desired_speed = self._clamp(angle_error, (-max_speed, max_speed))
        return self.to_speed(desired_speed, measured_speed, dt)


__all__ = ["PIDController"]
