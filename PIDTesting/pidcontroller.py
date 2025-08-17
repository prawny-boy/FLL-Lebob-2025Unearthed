class PIDController:
    """
    PID Controller for motor speed and position control.

    Supports:
    - Speed control
    - Angle control
    - Angle control with speed limiting

    Example
    -------
    >>> pid = PIDController(Kp=2.0, Ki=0.1, Kd=0.5)
    >>> power = pid.to_speed(target_speed=360, measured_speed=0, dt=0.01)
    >>> power = pid.to_angle(target_angle=720, measured_angle=0, dt=0.01)
    >>> power = pid.to_angle_with_speed(
    ...     target_angle=720,
    ...     measured_angle=0,
    ...     measured_speed=0,
    ...     max_speed=500,
    ...     dt=0.01
    ... )
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        """
        Initialize PID controller.

        Parameters
        ----------
        Kp : float
            Proportional gain.
        Ki : float
            Integral gain.
        Kd : float
            Derivative gain.

        Example
        -------
        >>> pid = PIDController(Kp=2.0, Ki=0.1, Kd=0.5)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = 0

    def reset(self):
        """
        Reset integral and derivative history.

        Example
        -------
        >>> pid.reset()
        """
        self.integral = 0
        self.last_error = 0

    def _calculate(self, error, dt):
        """
        Core PID calculation.

        Parameters
        ----------
        error : float
            The current error signal.
        dt : float
            Time step in seconds.

        Returns
        -------
        float
            Output clamped to [-100, 100].
        """
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return max(min(output, 100), -100)

    def to_speed(self, target_speed, measured_speed, dt):
        """
        PID for speed control.

        Parameters
        ----------
        target_speed : float
            Desired motor speed in deg/s.
        measured_speed : float
            Current motor speed in deg/s.
        dt : float
            Time step in seconds.

        Returns
        -------
        float
            Duty cycle (-100 to 100) to achieve target speed.
        """
        error = target_speed - measured_speed
        return self._calculate(error, dt)

    def to_angle(self, target_angle, measured_angle, dt):
        """
        PID for position control (angle).

        Parameters
        ----------
        target_angle : float
            Desired motor angle in degrees.
        measured_angle : float
            Current motor angle in degrees.
        dt : float
            Time step in seconds.

        Returns
        -------
        float
            Duty cycle (-100 to 100) to move toward target angle.
        """
        error = target_angle - measured_angle
        return self._calculate(error, dt)

    def to_angle_with_speed(self, target_angle, measured_angle, measured_speed, max_speed, dt):
        """
        Position PID with inner speed limiting loop.

        Parameters
        ----------
        target_angle : float
            Desired motor angle in degrees.
        measured_angle : float
            Current motor angle in degrees.
        measured_speed : float
            Current motor speed in deg/s.
        max_speed : float
            Maximum allowed speed in deg/s.
        dt : float
            Time step in seconds.

        Returns
        -------
        float
            Duty cycle (-100 to 100) respecting speed limit.
        """
        # Outer loop: angle error -> desired speed
        angle_error = target_angle - measured_angle
        desired_speed = max(min(angle_error, max_speed), -max_speed)

        # Inner loop: regulate speed toward desired speed
        return self.to_speed(desired_speed, measured_speed, dt)
