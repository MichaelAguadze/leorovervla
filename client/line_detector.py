"""Classical CV line detector for colored tape using HSV thresholding.

Supported colors: red, blue, green, white.

Each color is defined as a list of (lower, upper) HSV range pairs.
Multiple ranges per color are OR-ed together — this is required for red,
which wraps around the hue axis at 0/180 in OpenCV HSV encoding.
"""

from __future__ import annotations

import numpy as np

try:
    import cv2
except ImportError as exc:
    raise ImportError(
        "opencv-python-headless is required for line detection: "
        "pip install opencv-python-headless"
    ) from exc


# OpenCV HSV: H in [0, 180], S and V in [0, 255].
#
# Each entry is a list of (lower, upper) pairs. detect() ORs all pairs
# so colors that need two hue ranges (red) work transparently.
COLOR_PRESETS: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {
    "red": [
        # Lower red: H ~ 0°–20°
        (np.array([0,   100,  80], dtype=np.uint8),
         np.array([10,  255, 255], dtype=np.uint8)),
        # Upper red: H ~ 320°–360°
        (np.array([160, 100,  80], dtype=np.uint8),
         np.array([180, 255, 255], dtype=np.uint8)),
    ],
    "blue": [
        # H ~ 200°–260°
        (np.array([100, 100,  50], dtype=np.uint8),
         np.array([130, 255, 255], dtype=np.uint8)),
    ],
    "green": [
        # H ~ 80°–160°
        (np.array([40,  80,  50], dtype=np.uint8),
         np.array([80, 255, 255], dtype=np.uint8)),
    ],
    "white": [
        # Very low saturation, high value — any hue
        (np.array([0,   0,  200], dtype=np.uint8),
         np.array([180, 40, 255], dtype=np.uint8)),
    ],
}

SUPPORTED_COLORS = list(COLOR_PRESETS.keys())

_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


class LineDetector:
    """
    Detects a colored tape line in a 640x480 RGB frame.

    Steps:
      1. Crop to ROI (bottom 40% by default — nearest tape is most stable).
      2. Convert RGB → HSV.
      3. Apply color mask using preset HSV ranges (OR-ed if multiple ranges).
      4. Morphological open (noise removal) then dilate (connect fragments).
      5. Find largest contour; compute its centroid.
      6. Return normalized horizontal error and a found flag.

    Args:
        color: Tape color preset — one of "red", "blue", "green", "white".
        roi_top_fraction: Fraction from the top where the ROI begins.
                          0.60 keeps the bottom 40% of the frame.
        min_contour_area: Reject contours smaller than this pixel area.
    """

    def __init__(
        self,
        color: str = "red",
        roi_top_fraction: float = 0.60,
        min_contour_area: int = 500,
    ):
        if color not in COLOR_PRESETS:
            raise ValueError(
                f"Unknown tape color '{color}'. "
                f"Choose from: {', '.join(SUPPORTED_COLORS)}"
            )
        self.color = color
        self.roi_top_fraction = roi_top_fraction
        self.min_contour_area = min_contour_area
        self._ranges = COLOR_PRESETS[color]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, frame_rgb: np.ndarray) -> tuple[float, bool]:
        """
        Args:
            frame_rgb: (H, W, 3) uint8 RGB array from get_frame_rgb()

        Returns:
            (error, found)
            error: float in [-1, 1] — 0 = line centered,
                   positive = line is to the right.
            found: True if a usable centroid was detected.
        """
        roi = self._crop_roi(frame_rgb)
        w = roi.shape[1]
        mask = self._build_mask(roi)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, False

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_contour_area:
            return 0.0, False

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return 0.0, False

        cx = M["m10"] / M["m00"]
        error = (cx - w / 2.0) / (w / 2.0)
        return float(error), True

    def debug_mask(self, frame_rgb: np.ndarray) -> tuple[np.ndarray, int]:
        """Return the thresholded mask and its non-zero pixel count (for tuning)."""
        roi = self._crop_roi(frame_rgb)
        mask = self._build_mask(roi)
        return mask, int(np.count_nonzero(mask))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _crop_roi(self, frame_rgb: np.ndarray) -> np.ndarray:
        h = frame_rgb.shape[0]
        roi_start = int(h * self.roi_top_fraction)
        return frame_rgb[roi_start:, :]

    def _build_mask(self, roi_rgb: np.ndarray) -> np.ndarray:
        """Apply all HSV ranges for the selected color and return the combined mask."""
        hsv = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self._ranges[0][0], self._ranges[0][1])
        for lower, upper in self._ranges[1:]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   _KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, _KERNEL, iterations=2)
        return mask
