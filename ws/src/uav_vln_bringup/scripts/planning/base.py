"""planning/base.py — Planner ABC and Trajectory dataclass.

Contract for all planner implementations.  See docs/ARCHITECTURE.md §4.2.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point


@dataclass
class TrajectoryPoint:
    """Single point in a planned trajectory.

    Attributes:
        pos:           (x, y, z) position in meters.
        vel:           (vx, vy, vz) velocity in m/s.
        yaw:           Heading angle in radians.
        t_from_start:  Seconds from the beginning of the trajectory.
    """
    pos: Tuple[float, float, float]
    vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    t_from_start: float = 0.0


@dataclass
class Trajectory:
    """Full trajectory consisting of time-stamped waypoints.

    Attributes:
        points:          Ordered list of TrajectoryPoint.
        frame_id:        TF frame (usually "map").
        stamp_at_start:  ROS time (seconds) when trajectory playback began.
    """
    points: List[TrajectoryPoint] = field(default_factory=list)
    frame_id: str = "map"
    stamp_at_start: float = 0.0


class Planner(ABC):
    """Abstract base class for local trajectory planners.

    Implementations receive a start point and goal point and return a
    collision-free Trajectory (or None on failure).
    """

    @abstractmethod
    def plan(self, start: Point, goal: Point) -> Optional[Trajectory]:
        """Plan a trajectory from start to goal.

        Args:
            start: Current drone position.
            goal:  Target position.

        Returns:
            Trajectory on success, None if planning fails.
        """
        ...

    @abstractmethod
    def is_ready(self) -> bool:
        """Check whether the planner's internal state is ready.

        Typically means the map has been initialised and odometry is
        available.
        """
        ...

    def shutdown(self) -> None:
        """Clean up resources (sub-processes, subscriptions)."""
        pass
