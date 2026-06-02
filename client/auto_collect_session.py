"""Automated dataset collection session using classical CV line following."""

from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from config import RecordingConfig
from storage.episode_writer import EpisodeWriter
from storage.raw_writer import RawWriter
from timing import FPSRegulator

from .episode_manager import EpisodeManager
from .robot_client import RobotClient
from .line_detector import LineDetector
from .pd_controller import LinePDController


DIRECTION_OPTIONS = [
    ("clockwise", "clockwise"),
    ("counterclockwise", "counterclockwise"),
]


class AutoCollectSession:
    """
    Drives the leorover autonomously with a PD line-follower to collect
    CNN training episodes without a human operator.

    Episode lifecycle:
      1. Arm: countdown arm_delay_s so the user can place the robot on the line.
      2. Record: PD-control loop at config.fps until:
             - elapsed >= lap_time_s            → accept
             - line lost > line_lost_tolerance_s → discard
      3. Repeat until num_episodes accepted laps are saved.

    Output format is identical to CNNLoopSession (same parquet columns,
    same video structure) so auto and manual sessions can share a dataset root.
    """

    def __init__(
        self,
        config: RecordingConfig,
        direction: str = "clockwise",
        lap_time_s: float = 46.0,
        arm_delay_s: float = 5.0,
        line_lost_tolerance_s: float = 3.0,
        detector: LineDetector | None = None,
        controller: LinePDController | None = None,
        tape_color: str = "red",
    ):
        self.config = config
        self.direction = direction
        self.task_name = direction
        self.task_index = next(
            i for i, (d, _) in enumerate(DIRECTION_OPTIONS) if d == direction
        )
        self.lap_time_s = lap_time_s
        self.arm_delay_s = arm_delay_s
        self.line_lost_tolerance_s = line_lost_tolerance_s

        self.tape_color = tape_color
        self.detector = detector or LineDetector()
        self.controller = controller or LinePDController(max_duty=config.max_duty)

        self.config.dataset_dir.mkdir(parents=True, exist_ok=True)
        self.client = RobotClient(robot_url=config.robot_url, timeout=0.5, max_retries=1)
        self.episodes = EpisodeManager()
        self.fps_reg = FPSRegulator(target_fps=config.fps)

        self.session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        task_names = [d[1] for d in DIRECTION_OPTIONS]

        self.episode_writer = EpisodeWriter(
            episodes_dir=config.episodes_dir / self.session_name,
            fps=config.fps,
            vcodec=config.vcodec,
        )
        self.raw_writer = RawWriter(
            session_dir=config.raw_dir / self.session_name,
            fps=config.fps,
            vcodec=config.vcodec,
        )

        self._running = False
        self._last_health_check = 0.0
        self._task_names = task_names
        self._write_session_info()

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Collect config.num_episodes accepted laps unattended."""
        print()
        print("=" * 50)
        print("  leorover Automated Dataset Collector")
        print("=" * 50)

        if not self.client.is_connected():
            print(f"  ERROR: Cannot reach robot at {self.config.robot_url}")
            print("  Make sure the robot server is running.")
            return

        health = self.client.get_health()
        print(
            f"  Connected! Battery: {health.get('battery_mv', '?')}mV, "
            f"Camera: {'OK' if health.get('camera_ok') else 'FAIL'}"
        )

        if not self.episode_writer.video_available:
            print("  ERROR: PyAV is required for MP4. Install with: pip install av")
            return

        self.raw_writer.start()
        self.episode_writer.save_task_mapping(self._task_names)
        self._running = True

        print(f"\n  Target : {self.config.num_episodes} accepted laps ({self.direction})")
        print(f"  Lap    : {self.lap_time_s:.0f}s  |  Line-lost: {self.line_lost_tolerance_s:.1f}s  |  Arm: {self.arm_delay_s:.0f}s")
        print("  Press Ctrl+C to stop early.\n")

        try:
            while self._running and self.episodes.accepted_count < self.config.num_episodes:
                self._arm_phase()
                if not self._running:
                    break
                accepted = self._record_episode()
                if accepted:
                    print(
                        f"  Total accepted: {self.episodes.accepted_count} / "
                        f"{self.config.num_episodes} episodes\n"
                    )
        except KeyboardInterrupt:
            print("\n\n  Ctrl+C — stopping...")
        finally:
            self._shutdown()

    # ------------------------------------------------------------------
    # Internal phases
    # ------------------------------------------------------------------

    def _arm_phase(self) -> None:
        """Countdown so the user can place the robot on the line."""
        for remaining in range(int(self.arm_delay_s), 0, -1):
            print(f"\r  [ARM] Place robot on line — starting in {remaining}s... ", end="", flush=True)
            time.sleep(1.0)
        print("\r  [ARM] Go!                                          ")

    def _record_episode(self) -> bool:
        """Drive one lap autonomously; return True if accepted."""
        self.episodes.start_episode(self.task_name, self.task_index)
        self.fps_reg.reset()
        self.controller.reset()

        ep_idx = self.episodes.current.episode_index
        print(f"\n  RECORDING Episode {ep_idx}  [{self.direction}]  (max {self.lap_time_s:.0f}s)")

        start_time = time.monotonic()
        frame_count = 0
        moving_frames = 0
        previous_action = np.zeros(3, dtype=np.float32)
        last_line_seen = start_time
        discard_reason: str = ""

        while self._running:
            self.fps_reg.tick()
            self._check_health()

            elapsed = time.monotonic() - start_time

            if elapsed >= self.lap_time_s:
                break

            try:
                image_rgb, robot_ts, _ = self.client.get_frame_rgb()
            except Exception as exc:
                print(f"\r  [WARN] Frame grab failed: {exc}   ", end="", flush=True)
                continue

            error, found = self.detector.detect(image_rgb)
            if found:
                last_line_seen = time.monotonic()

            line_lost_s = time.monotonic() - last_line_seen
            if line_lost_s > self.line_lost_tolerance_s:
                discard_reason = f"line lost for {line_lost_s:.1f}s"
                break

            vx, vy, omega = self.controller.compute(error, found)
            action = np.array([vx, vy, omega], dtype=np.float32) / self.config.max_duty
            state = previous_action.copy()

            try:
                sent = self.client.send_velocity(vx, vy, omega)
            except Exception as exc:
                print(f"\r  [WARN] Motor command failed: {exc}   ", end="", flush=True)
                continue

            if not sent:
                print("\r  [WARN] Robot rejected velocity command.   ", end="", flush=True)
                continue

            episode_ts = frame_count / self.config.fps
            self.episodes.add_frame(image_rgb, state, action, episode_ts)
            self.raw_writer.write_frame(
                image=image_rgb,
                state=state,
                action=action,
                timestamp=robot_ts,
                task=self.task_name,
                episode_index=ep_idx,
            )

            previous_action = action.copy()
            if not np.allclose(action, 0.0, atol=1e-6):
                moving_frames += 1
            frame_count += 1

            fps_str = f"{self.fps_reg.actual_fps:.1f}" if frame_count > 2 else "..."
            status = "TRACK" if found else "LOST!"
            print(
                f"\r  REC {elapsed:5.1f}s/{self.lap_time_s:.0f}s  "
                f"frames={frame_count}  fps={fps_str}  "
                f"err={error:+.2f}  [{status}]   ",
                end="",
                flush=True,
            )

        self.client.stop()
        print()

        if discard_reason:
            self.episodes.discard_episode()
            print(f"  x Episode {ep_idx} discarded: {discard_reason}\n")
            return False
        if frame_count < 5:
            self.episodes.discard_episode()
            print(f"  x Episode {ep_idx} too short ({frame_count} frames), discarded.\n")
            return False
        if moving_frames < 3:
            self.episodes.discard_episode()
            print(f"  x Episode {ep_idx} too little movement ({moving_frames} frames), discarded.\n")
            return False

        episode = self.episodes.accept_episode()
        episode_dir = self.episode_writer.save_episode(episode)
        self._write_episode_info(episode_dir=episode_dir, episode=episode)
        print(
            f"  ok Episode {ep_idx} accepted "
            f"({frame_count} frames, {frame_count / self.config.fps:.1f}s)\n"
        )
        return True

    # ------------------------------------------------------------------
    # Metadata helpers
    # ------------------------------------------------------------------

    def _write_session_info(self) -> None:
        session_info = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "session_name": self.session_name,
            "dataset_name": self.config.dataset_name,
            "robot_url": self.config.robot_url,
            "robot_type": self.config.robot_type,
            "fps": self.config.fps,
            "episode_time_s": self.lap_time_s,
            "max_duty": self.config.max_duty,
            "teleop_speed": self.config.teleop_speed,
            "vcodec": self.config.vcodec,
            "mode_family": "cnn",
            "intent_mode": "no_language",
            "task_type": "path_following",
            "track_layout": "user_defined",
            "allowed_directions": [d[0] for d in DIRECTION_OPTIONS],
            "episode_definition": "one_full_lap_auto_accept",
            "collection_style": "clean_lap",
            "collector": "auto_cv_pd",
            "tape_color": self.tape_color,
            "observation_state_semantics": "previous_action_normalized",
            "action_semantics": "current_action_normalized",
            "accepted_episode_timestamps": "episode_relative_seconds",
            "raw_backup_timestamps": "robot_monotonic_seconds",
        }
        for base_dir in (self.config.raw_dir, self.config.episodes_dir):
            session_dir = base_dir / self.session_name
            session_dir.mkdir(parents=True, exist_ok=True)
            path = session_dir / "session_info.json"
            with path.open("w", encoding="utf-8") as handle:
                json.dump(session_info, handle, indent=2)

    def _write_episode_info(self, episode_dir: Path, episode) -> None:
        info = {
            "episode_index": episode.episode_index,
            "direction": self.direction,
            "mode_family": "cnn",
            "intent_mode": "no_language",
            "task_type": "path_following",
            "track_layout": "user_defined",
            "episode_definition": "one_full_lap_auto_accept",
            "collection_style": "clean_lap",
            "tape_color": self.tape_color,
            "task_name": episode.task,
            "task_index": episode.task_index,
            "num_frames": len(episode.frames),
            "duration_s": len(episode.frames) / self.config.fps,
            "collector": "auto_cv_pd",
        }
        with (episode_dir / "episode_info.json").open("w", encoding="utf-8") as handle:
            json.dump(info, handle, indent=2)

    def _check_health(self) -> None:
        now = time.monotonic()
        if now - self._last_health_check < 30:
            return
        self._last_health_check = now
        try:
            health = self.client.get_health()
        except Exception:
            return
        battery_mv = health.get("battery_mv", 0)
        if battery_mv and battery_mv < 7200:
            print(f"\n  [WARN] Battery is low ({battery_mv}mV). Consider charging soon.")
        if not health.get("camera_ok", True):
            print("\n  [WARN] Robot reports camera problems.")

    def _shutdown(self) -> None:
        print("\n  Shutting down...")
        try:
            self.client.stop()
        except Exception:
            pass
        if self.episodes.is_recording:
            self.episodes.discard_episode()
            print("  Discarded in-progress episode.")
        self.raw_writer.close()
        print(f"\n  Session complete: {self.session_name}")
        print(f"    Accepted episodes : {self.episodes.accepted_count}")
        print(f"    Total frames      : {self.episodes.total_frames}")
        print(f"    Episodes          : {self.config.episodes_dir / self.session_name}")
        print(f"    Raw data          : {self.config.raw_dir / self.session_name}")
        print()
