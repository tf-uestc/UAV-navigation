"""
dummy_detector.py — Dummy detector for testing the node skeleton.

Always returns a fixed 100×100 box at the image center, with median depth
from valid pixels within that bounding box.
"""
from typing import Optional

import numpy as np
from sensor_msgs.msg import CameraInfo

from base_detector import Detection, Detector


def pixel_to_camera_optical(u: float, v: float, depth: float,
                            fx: float, fy: float, cx: float, cy: float):
    """Back-project a pixel to 3D camera optical frame coordinates.

    Args:
        u, v: Pixel coordinates.
        depth: Depth in meters at that pixel.
        fx, fy, cx, cy: Camera intrinsics.

    Returns:
        (x, y, z) tuple in camera optical frame (x-right, y-down, z-forward).
    """
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return (x, y, z)


class DummyDetector(Detector):
    """Always detects a fixed 100×100 box at the image center.

    Depth is computed as the median of valid (>0, not nan, finite) depth
    pixels within the bounding box.  Returns None if there are no valid
    depth pixels in the box.
    """

    def detect(self,
               rgb: np.ndarray,
               depth: np.ndarray,
               target_text: str,
               cam_info: CameraInfo) -> Optional[Detection]:
        """Return a centered 100×100 Detection, or None on failure."""
        h, w = depth.shape[:2]

        # --- 1. Compute clamped bounding box ---
        x1 = max(0, w // 2 - 50)
        y1 = max(0, h // 2 - 50)
        x2 = min(w - 1, w // 2 + 50)
        y2 = min(h - 1, h // 2 + 50)
        bbox = (x1, y1, x2, y2)

        # --- 2. Extract depth region ---
        region = depth[y1:y2 + 1, x1:x2 + 1]
        total_pixels = region.size

        # --- 3. Filter valid pixels ---
        valid = region[(region > 0) & np.isfinite(region)]
        valid_count = valid.size

        if valid_count == 0:
            return None

        # --- 4. Compute depth & confidence ---
        depth_m = float(np.median(valid))
        confidence = valid_count / total_pixels

        # --- 5. Back-project to camera optical frame ---
        K = cam_info.K  # row-major 3x3 intrinsics
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # center uses the same integer midpoint that Detection.__post_init__ computes
        center_u = (x1 + x2) / 2.0
        center_v = (y1 + y2) / 2.0
        point_camera = pixel_to_camera_optical(center_u, center_v, depth_m,
                                               fx, fy, cx, cy)

        # --- 6. Assemble Detection ---
        return Detection(
            bbox=bbox,
            point_camera=point_camera,
            depth_m=depth_m,
            confidence=confidence,
            target_text=target_text,
            raw={"type": "dummy", "img_shape": (h, w)},
        )
