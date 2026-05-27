"""PD controller for holonomic line following."""

from __future__ import annotations


class LinePDController:
    """
    Maps normalized centroid error to (vx, vy, omega) duty-cycle commands.

    Control law:
        omega = -(Kp * error + Kd * d_error)   [clamped to ±max_duty]
        vx    = base_speed * (1 - speed_damp * |error|)
        vy    = 0.0  (strafe unused — yaw correction is sufficient)

    When found=False (line not visible), returns (0, 0, 0) so the robot
    stops in place rather than driving blind.
    """

    def __init__(
        self,
        Kp: float = 45.0,
        Kd: float = 8.0,
        base_speed: float = 35.0,
        speed_damp: float = 0.5,
        max_duty: float = 80.0,
    ):
        self.Kp = Kp
        self.Kd = Kd
        self.base_speed = base_speed
        self.speed_damp = speed_damp
        self.max_duty = max_duty
        self._prev_error: float = 0.0

    def compute(self, error: float, found: bool) -> tuple[float, float, float]:
        """
        Args:
            error: normalized centroid error in [-1, 1]
            found: whether the line was detected this tick

        Returns:
            (vx, vy, omega) in duty-cycle units, clamped to ±max_duty
        """
        if not found:
            self._prev_error = 0.0
            return 0.0, 0.0, 0.0

        d_error = error - self._prev_error
        self._prev_error = error

        omega = -(self.Kp * error + self.Kd * d_error)
        omega = max(-self.max_duty, min(self.max_duty, omega))

        vx = self.base_speed * (1.0 - self.speed_damp * abs(error))
        vx = max(0.0, vx)

        return vx, 0.0, omega

    def reset(self) -> None:
        self._prev_error = 0.0
