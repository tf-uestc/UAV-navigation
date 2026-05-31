"""
base_detector.py — Detection dataclass and Detector ABC.

This is the contract all detector implementations must follow.
See docs/ARCHITECTURE.md §4.1 for the design rationale.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
from sensor_msgs.msg import CameraInfo


@dataclass
class Detection:
    """Single-frame detection result.

    All position fields follow ROS conventions:
    - camera optical frame: z-forward, x-right, y-down
    - image frame: (u,v) with origin at top-left

    Attributes:
        bbox: (x1, y1, x2, y2) pixel coordinates in image frame.
        point_camera: (x, y, z) 3D position in camera optical frame (meters).
        depth_m: Depth in meters at the detection point.
        confidence: Detection confidence in [0, 1].
        raw: Raw provider response for debugging.
        target_text: The target semantic text that was queried.
        center: (u, v) pixel center, auto-computed from bbox.
    """
    bbox: Tuple[int, int, int, int]
    point_camera: Tuple[float, float, float]
    depth_m: float
    confidence: float
    raw: dict = field(default_factory=dict)
    target_text: str = ""
    center: Tuple[int, int] = field(init=False)

    def __post_init__(self) -> None:
        self.center = (
            (self.bbox[0] + self.bbox[2]) // 2,
            (self.bbox[1] + self.bbox[3]) // 2,
        )


class Detector(ABC):
    """Abstract base class for all VLM/vision-based detectors.

    Implementations receive synchronized RGB+Depth frames plus camera
    intrinsics and return a Detection (or None on failure).
    """

    @abstractmethod
    def detect(self,
               rgb: np.ndarray,
               depth: np.ndarray,
               target_text: str,
               cam_info: CameraInfo) -> Optional[Detection]:
        """Detect a target in a single frame.

        Args:
            rgb: RGB image as numpy array, shape (H, W, 3), dtype uint8.
            depth: Depth image as numpy array, shape (H, W), dtype float32
                   in meters. Aligned to rgb.
            target_text: Semantic description of the target to detect
                         (e.g. "a red car", "survivor in orange jacket").
            cam_info: Camera intrinsics (sensor_msgs/CameraInfo) for
                      3D back-projection.

        Returns:
            Detection on success, or None if the target was not found.
        """
        ...

    def healthcheck(self) -> bool:
        """Check whether the detector backend is reachable and ready.

        Returns:
            True if the backend is healthy, False otherwise.
        """
        return True
