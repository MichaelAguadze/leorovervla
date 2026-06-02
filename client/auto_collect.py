"""Automated CNN dataset collector via CV line following.

Usage:
    python -m client.auto_collect --robot-ip <IP> --episodes <N>

The robot drives autonomously around a red-tape track. Each lap is
accepted automatically after --lap-time seconds, or discarded if the
line is lost for longer than --line-lost-tolerance seconds.

Output is written to the same dataset format as the manual CNN recorder
so both sessions can be mixed under the same --dataset root.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m client.auto_collect",
        description="Automated CNN dataset collector using CV line following",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Connection
    conn = p.add_argument_group("connection")
    conn.add_argument("--robot-ip",   default="192.168.2.12")
    conn.add_argument("--robot-port", type=int, default=8081)

    # Dataset
    ds = p.add_argument_group("dataset")
    ds.add_argument("--dataset",  default="leorover_cnn_red",
                    help="Dataset name (subdirectory under --data-dir)")
    ds.add_argument("--data-dir", default="data")
    ds.add_argument("--episodes", type=int, default=20,
                    help="Number of accepted laps to collect")
    ds.add_argument("--direction",
                    choices=["clockwise", "counterclockwise"],
                    default="clockwise")
    ds.add_argument("--fps", type=int, default=10)

    # Timing
    timing = p.add_argument_group("timing")
    timing.add_argument("--lap-time",            type=float, default=46.0,
                        help="Seconds per lap before auto-accept")
    timing.add_argument("--arm-delay",           type=float, default=5.0,
                        help="Countdown seconds before each lap starts")
    timing.add_argument("--line-lost-tolerance", type=float, default=3.0,
                        help="Seconds of missing line before discarding an episode")

    # PD gains
    pd = p.add_argument_group("PD controller")
    pd.add_argument("--Kp",         type=float, default=45.0,
                    help="Proportional gain for omega")
    pd.add_argument("--Kd",         type=float, default=8.0,
                    help="Derivative gain for omega")
    pd.add_argument("--base-speed", type=float, default=35.0,
                    help="Base forward speed in duty-cycle units (0–80)")
    pd.add_argument("--speed-damp",   type=float, default=0.5,
                    help="Speed reduction fraction at max error (0=none, 1=full stop)")
    pd.add_argument("--coast-factor",         type=float, default=0.6,
                    help="Power fraction to replay last omega when coasting a curve")
    pd.add_argument("--corner-omega",         type=float, default=None,
                    help="Fixed omega duty at corners (auto-derived from --direction if omitted; "
                         "negative=right/CW, positive=left/CCW)")
    pd.add_argument("--coast-omega-threshold",     type=float, default=5.0,
                    help="|last_omega| below this triggers corner mode instead of coasting")
    pd.add_argument("--corner-approach-threshold", type=float, default=0.75,
                    help="Centroid error magnitude at which predictive corner turn starts "
                         "(0–1, lower = earlier trigger)")
    pd.add_argument("--max-duty",             type=float, default=80.0,
                    help="Duty-cycle ceiling for all commands")

    # Detection
    det = p.add_argument_group("line detection")
    det.add_argument("--tape-color",
                     choices=["red", "blue", "green", "white"],
                     default="red",
                     help="Tape color preset for HSV thresholding")
    det.add_argument("--roi-top",     type=float, default=0.60,
                     help="Fraction from top where ROI begins (0.6 = bottom 40%%)")
    det.add_argument("--min-contour", type=int,   default=500,
                     help="Minimum contour area in pixels to count as line")

    return p


def main() -> None:
    args = _build_parser().parse_args()

    from config import RecordingConfig
    from .line_detector import LineDetector
    from .pd_controller import LinePDController
    from .auto_collect_session import AutoCollectSession

    config = RecordingConfig(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        dataset_name=args.dataset,
        fps=args.fps,
        num_episodes=args.episodes,
        episode_time_s=args.lap_time,
        max_duty=args.max_duty,
        data_dir=Path(args.data_dir),
    )

    detector = LineDetector(
        color=args.tape_color,
        roi_top_fraction=args.roi_top,
        min_contour_area=args.min_contour,
    )
    # Auto-derive corner_omega from direction unless the user overrides it.
    # Clockwise path → right turns at corners → negative omega.
    # Counterclockwise path → left turns at corners → positive omega.
    if args.corner_omega is not None:
        corner_omega = args.corner_omega
    elif args.direction == "clockwise":
        corner_omega = -args.max_duty * 0.5
    else:
        corner_omega = args.max_duty * 0.5

    controller = LinePDController(
        Kp=args.Kp,
        Kd=args.Kd,
        base_speed=args.base_speed,
        speed_damp=args.speed_damp,
        coast_factor=args.coast_factor,
        corner_omega=corner_omega,
        coast_omega_threshold=args.coast_omega_threshold,
        corner_approach_threshold=args.corner_approach_threshold,
        max_duty=args.max_duty,
    )

    session = AutoCollectSession(
        config=config,
        direction=args.direction,
        lap_time_s=args.lap_time,
        arm_delay_s=args.arm_delay,
        line_lost_tolerance_s=args.line_lost_tolerance,
        detector=detector,
        controller=controller,
        tape_color=args.tape_color,
    )
    session.run()


if __name__ == "__main__":
    main()
