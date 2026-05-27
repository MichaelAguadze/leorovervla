"""PD controller for holonomic line following."""

from __future__ import annotations


class LinePDController:
    """
    Maps normalized centroid error to (vx, vy, omega) duty-cycle commands.

    Control law (line visible):
        omega = -(Kp * error + Kd * d_error)   [clamped to ±max_duty]
        vx    = base_speed * (1 - speed_damp * |error|)
        vy    = 0.0

    Line-lost behaviour (two cases):
        1. Robot was actively turning (|last_omega| >= coast_omega_threshold):
           Coast — replay last_omega * coast_factor.  Handles the robot
           drifting off a curve mid-segment.

        2. Robot was going straight (|last_omega| < coast_omega_threshold):
           Corner — use corner_omega at reduced forward speed.  This is the
           common case for rectangular paths: the robot arrives at a corner
           going straight, the tape exits the frame, and a fixed-rate spin in
           the correct direction sweeps the camera back onto the next segment.

    Args:
        corner_omega: Fixed omega duty to use when a corner is detected.
                      Sign encodes direction:
                        negative → right turn  (clockwise path)
                        positive → left turn   (counterclockwise path)
                      Set to 0.0 to fall back to pure coasting everywhere.
        coast_omega_threshold: |last_omega| below this value is treated as
                      "going straight" and triggers the corner turn.
        coast_factor: Scale applied to last_omega during curve coasting.
    """

    def __init__(
        self,
        Kp: float = 45.0,
        Kd: float = 8.0,
        base_speed: float = 35.0,
        speed_damp: float = 0.5,
        max_duty: float = 80.0,
        coast_factor: float = 0.6,
        corner_omega: float = 0.0,
        coast_omega_threshold: float = 5.0,
        corner_approach_threshold: float = 0.75,
    ):
        self.Kp = Kp
        self.Kd = Kd
        self.base_speed = base_speed
        self.speed_damp = speed_damp
        self.max_duty = max_duty
        self.coast_factor = coast_factor
        self.corner_omega = corner_omega
        self.coast_omega_threshold = coast_omega_threshold
        self.corner_approach_threshold = corner_approach_threshold
        self._prev_error: float = 0.0
        self._last_omega: float = 0.0

    def compute(self, error: float, found: bool) -> tuple[float, float, float]:
        """
        Args:
            error: normalized centroid error in [-1, 1]
            found: whether the line was detected this tick

        Returns:
            (vx, vy, omega) in duty-cycle units, clamped to ±max_duty
        """
        if not found:
            if abs(self._last_omega) < self.coast_omega_threshold and self.corner_omega != 0.0:
                # Robot was going straight when line disappeared — corner detected.
                # Spin at the preset corner rate with reduced forward speed.
                return self.base_speed * 0.4, 0.0, self.corner_omega
            else:
                # Robot was mid-curve — coast through.
                omega = self._last_omega * self.coast_factor
                vx = self.base_speed * self.coast_factor if abs(omega) > 1.0 else 0.0
                return vx, 0.0, omega

        # Predictive corner: line is still visible but centroid is near the frame
        # edge — corner is imminent. Start the corner turn NOW before the tape
        # exits the frame entirely.
        if self.corner_omega != 0.0 and abs(error) > self.corner_approach_threshold:
            self._prev_error = error
            self._last_omega = self.corner_omega
            vx = self.base_speed * 0.4
            return vx, 0.0, self.corner_omega

        d_error = error - self._prev_error
        self._prev_error = error

        omega = -(self.Kp * error + self.Kd * d_error)
        omega = max(-self.max_duty, min(self.max_duty, omega))

        vx = self.base_speed * (1.0 - self.speed_damp * abs(error))
        vx = max(0.0, vx)

        self._last_omega = omega
        return vx, 0.0, omega

    def reset(self) -> None:
        self._prev_error = 0.0
        self._last_omega = 0.0
