"""planning/dummy_planner.py — Straight-line dummy planner for testing.

Always returns a 30-point linear-interpolation trajectory from start
to goal.  Used to validate the planner_node → trajectory_follower →
setpoint_bridge pipeline before integrating a real planner.
"""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Point

from planning.base import Planner, Trajectory, TrajectoryPoint


def _distance(a: Tuple[float, float, float],
              b: Tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class DummyPlanner(Planner):
    """Dummy planner — straight line at 1.0 m/s for testing."""

    def __init__(self, num_points: int = 30, speed: float = 1.0) -> None:
        self._num_points = num_points
        self._speed = speed

    def is_ready(self) -> bool:
        return True

    def plan(self, start: Point, goal: Point) -> Optional[Trajectory]:
        sx, sy, sz = start.x, start.y, start.z
        gx, gy, gz = goal.x, goal.y, goal.z

        dist = _distance((sx, sy, sz), (gx, gy, gz))
        if dist < 0.01:
            # Already at goal — return a single-point trajectory
            return Trajectory(
                points=[TrajectoryPoint(pos=(gx, gy, gz), t_from_start=0.0)],
                frame_id="map",
            )

        total_time = dist / self._speed
        dt = total_time / (self._num_points - 1)

        points: list[TrajectoryPoint] = []
        for i in range(self._num_points):
            t = i * dt
            alpha = t / total_time if total_time > 0 else 1.0
            x = sx + (gx - sx) * alpha
            y = sy + (gy - sy) * alpha
            z = sz + (gz - sz) * alpha

            yaw = math.atan2(gy - sy, gx - sx)

            points.append(
                TrajectoryPoint(
                    pos=(x, y, z),
                    vel=(self._speed * (gx - sx) / dist,
                         self._speed * (gy - sy) / dist,
                         self._speed * (gz - sz) / dist),
                    yaw=yaw,
                    t_from_start=t,
                )
            )

        return Trajectory(points=points, frame_id="map")
