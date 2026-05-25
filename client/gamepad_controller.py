"""PS5 / generic gamepad teleop controller using pygame.

Default button mapping for PS5 DualSense (macOS / Linux Bluetooth):
    Left stick Y  (axis 1) — vx  forward / backward  (inverted)
    Left stick X  (axis 0) — vy  strafe left / right  (inverted)
    Right stick X (axis 2) — omega rotation
    Cross     (btn 0) — accept episode / confirm prompt
    Circle    (btn 1) — discard episode
    Square    (btn 2) — speed down
    Triangle  (btn 3) — speed up
    Options   (btn 9) — stop session

Axis indices and button numbers can vary by OS and driver version.
Override them with constructor keyword arguments if your mapping differs.
"""
from __future__ import annotations

import threading
import time

import numpy as np

try:
    import pygame
except ImportError:
    raise ImportError("pygame is required for gamepad support: pip install pygame")

_SPEED_REPEAT_HZ = 4.0  # speed steps per second while button held


class GamepadController:
    """Non-blocking PS5/gamepad teleop controller.

    Implements the same interface as TeleopController so it can be used
    as a drop-in replacement in all recording sessions.
    """

    def __init__(
        self,
        speed: float = 50.0,
        max_speed: float = 100.0,
        min_speed: float = 10.0,
        speed_step: float = 10.0,
        joystick_index: int = 0,
        deadzone: float = 0.12,
        axis_vx: int = 1,
        axis_vy: int = 0,
        axis_omega: int = 2,
        btn_accept: int = 0,
        btn_discard: int = 1,
        btn_speed_down: int = 2,
        btn_speed_up: int = 3,
        btn_stop: int = 9,
    ):
        self.speed = speed
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.speed_step = speed_step
        self.joystick_index = joystick_index
        self.deadzone = deadzone

        self._axis_vx = axis_vx
        self._axis_vy = axis_vy
        self._axis_omega = axis_omega
        self._btn_accept = btn_accept
        self._btn_discard = btn_discard
        self._btn_speed_up = btn_speed_up
        self._btn_speed_down = btn_speed_down
        self._btn_stop = btn_stop

        self._joystick: pygame.joystick.Joystick | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._speed_up_next: float = 0.0
        self._speed_down_next: float = 0.0

        self.events: dict[str, bool] = {
            "accept_episode": False,
            "discard_episode": False,
            "stop_session": False,
            "enter_pressed": False,
        }

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Initialize pygame joystick and start the background poll thread."""
        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError(
                "No gamepad detected. Connect your PS5 controller and try again."
            )

        self._joystick = pygame.joystick.Joystick(self.joystick_index)
        self._joystick.init()
        print(f"  Gamepad: {self._joystick.get_name()} (index {self.joystick_index})")

        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the poll thread and shut down pygame."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            pygame.quit()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal poll loop
    # ------------------------------------------------------------------

    def _deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _poll_loop(self) -> None:
        interval = 1.0 / _SPEED_REPEAT_HZ
        while self._running:
            pygame.event.pump()

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    btn = event.button
                    if btn == self._btn_accept:
                        self.events["accept_episode"] = True
                        self.events["enter_pressed"] = True
                    elif btn == self._btn_discard:
                        self.events["discard_episode"] = True
                    elif btn == self._btn_stop:
                        self.events["stop_session"] = True

            js = self._joystick
            if js is None:
                time.sleep(0.01)
                continue

            now = time.monotonic()

            if js.get_button(self._btn_speed_up) and now >= self._speed_up_next:
                self.speed = min(self.max_speed, self.speed + self.speed_step)
                self._speed_up_next = now + interval

            if js.get_button(self._btn_speed_down) and now >= self._speed_down_next:
                self.speed = max(self.min_speed, self.speed - self.speed_step)
                self._speed_down_next = now + interval

            # Left stick Y is inverted: push up → negative → forward
            vx = self._deadzone(-js.get_axis(self._axis_vx)) * self.speed
            vy = self._deadzone(-js.get_axis(self._axis_vy)) * self.speed
            omega = self._deadzone(js.get_axis(self._axis_omega)) * self.speed

            with self._lock:
                self._vx = vx
                self._vy = vy
                self._omega = omega

            time.sleep(0.01)

    # ------------------------------------------------------------------
    # Public interface (matches TeleopController)
    # ------------------------------------------------------------------

    def clear_events(self) -> None:
        for k in self.events:
            self.events[k] = False

    def wait_for_enter(self) -> None:
        """Block until Cross (X) is pressed or the session is stopped."""
        self.events["enter_pressed"] = False
        while not self.events["enter_pressed"] and not self.events["stop_session"]:
            time.sleep(0.05)
        self.events["enter_pressed"] = False

    def get_action(self) -> tuple[float, float, float]:
        """Return (vx, vy, omega) in duty-cycle units."""
        with self._lock:
            return self._vx, self._vy, self._omega

    def get_normalized_action(self, duty_range: float = 80.0) -> np.ndarray:
        """Return action normalized to [-1, 1]."""
        vx, vy, omega = self.get_action()
        return np.array(
            [vx / duty_range, vy / duty_range, omega / duty_range],
            dtype=np.float32,
        )
