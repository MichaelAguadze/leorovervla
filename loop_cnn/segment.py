"""Segment recorded episode videos to isolate the duct tape.

Reads each episode's video.mp4, applies an HSV tape-color mask, zeroes out the
background, and writes video_masked.mp4 next to the original.

Usage:
    python -m loop_cnn.segment [--episodes-dir PATH] [--tape-color COLOR] [--overwrite]
"""

from __future__ import annotations

import argparse
from fractions import Fraction
from pathlib import Path

import numpy as np

try:
    import av
except ImportError as exc:
    raise RuntimeError("PyAV is required: pip install av") from exc

try:
    import cv2
except ImportError as exc:
    raise RuntimeError("OpenCV is required: pip install opencv-python-headless") from exc

from client.line_detector import COLOR_PRESETS
from . import DEFAULT_DATA_ROOT
from .dataset import discover_cnn_episodes

_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def build_tape_mask(frame_rgb: np.ndarray, color: str) -> np.ndarray:
    """Return a binary uint8 mask (255 = tape, 0 = background) for one RGB frame."""
    ranges = COLOR_PRESETS[color]
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, ranges[0][0], ranges[0][1])
    for lower, upper in ranges[1:]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   _MORPH_KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, _MORPH_KERNEL, iterations=2)
    return mask


def apply_tape_mask(frame_rgb: np.ndarray, color: str) -> np.ndarray:
    """Zero out background pixels, keep tape pixels at their original colour."""
    out = frame_rgb.copy()
    out[build_tape_mask(frame_rgb, color) == 0] = 0
    return out


def segment_video(input_path: Path, output_path: Path, color: str = "red") -> int:
    """Write a background-masked copy of input_path to output_path.

    Returns the number of frames processed.
    """
    with av.open(str(input_path)) as in_container:
        in_stream = in_container.streams.video[0]
        fps = Fraction(in_stream.average_rate)
        width = in_stream.width
        height = in_stream.height

        with av.open(str(output_path), mode="w") as out_container:
            out_stream = out_container.add_stream("libx264", rate=fps)
            out_stream.width = width
            out_stream.height = height
            out_stream.pix_fmt = "yuv420p"
            out_stream.options = {"crf": "23", "preset": "fast"}

            n = 0
            for in_frame in in_container.decode(in_stream):
                rgb = in_frame.to_ndarray(format="rgb24")
                masked = apply_tape_mask(rgb, color)

                out_frame = av.VideoFrame.from_ndarray(masked, format="rgb24")
                out_frame.pts = in_frame.pts
                out_frame.time_base = in_frame.time_base

                for packet in out_stream.encode(out_frame):
                    out_container.mux(packet)
                n += 1

            for packet in out_stream.encode():
                out_container.mux(packet)

    return n


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Segment duct tape in recorded CNN episode videos."
    )
    parser.add_argument(
        "--episodes-dir",
        default=DEFAULT_DATA_ROOT,
        help=f"Root episodes directory (default: {DEFAULT_DATA_ROOT})",
    )
    parser.add_argument(
        "--tape-color",
        choices=list(COLOR_PRESETS.keys()),
        default="red",
        help="Tape color to isolate (default: red)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Re-process episodes that already have a masked video.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    episodes_dir = Path(args.episodes_dir)

    records = discover_cnn_episodes(episodes_dir, tape_color=args.tape_color)
    if not records:
        print(f"No {args.tape_color} episodes found under {episodes_dir}")
        return

    print(f"Found {len(records)} {args.tape_color} episodes under {episodes_dir}")
    skipped = processed = 0

    for record in records:
        in_path = record.episode_dir / "video.mp4"
        out_path = record.episode_dir / "video_masked.mp4"

        if out_path.exists() and not args.overwrite:
            skipped += 1
            continue

        rel = record.episode_dir.relative_to(episodes_dir)
        print(f"  {rel} ... ", end="", flush=True)
        n = segment_video(in_path, out_path, args.tape_color)
        print(f"{n} frames")
        processed += 1

    print(f"\nDone. Processed: {processed}  Skipped: {skipped}")


if __name__ == "__main__":
    main()
