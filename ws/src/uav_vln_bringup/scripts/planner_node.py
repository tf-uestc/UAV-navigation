#!/usr/bin/env python3
"""planner_node.py — ROS planner node with importlib-based Planner loading.

Subscribes to the F2 target and drone pose, runs planning, publishes
trajectory as nav_msgs/Path and status as std_msgs/String.

Params:
  ~planner_class  (str)   Dotted path to Planner subclass (default
                           planning.dummy_planner.DummyPlanner).
  ~planner_args   (dict)  Keyword arguments forwarded to the planner
                           constructor.
"""

import importlib
import os
import sys

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header, String

# Ensure scripts/ is on PYTHONPATH
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from planning.base import Planner, Trajectory  # noqa: E402


class PlannerNode:
    """ROS node wrapping a Planner implementation."""

    def __init__(self) -> None:
        rospy.init_node("planner_node")
        self._status_msg = String()

        # --- Load planner via importlib ---
        cls_path = rospy.get_param(
            "~planner_class", "planning.dummy_planner.DummyPlanner"
        )
        planner_args = rospy.get_param("~planner_args", {})
        self.planner: Planner = self._load_planner(cls_path, planner_args)
        rospy.loginfo("[planner] Loaded planner: %s", cls_path)

        # --- State ---
        self._current_pose: PoseStamped = PoseStamped()
        self._mission_ok = True

        # --- Subscribers ---
        rospy.Subscriber(
            "/uav/target_world", PointStamped,
            self._on_target, queue_size=1,
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped,
            self._on_pose, queue_size=10,
        )
        rospy.Subscriber(
            "/uav/state_cmd", String,
            self._on_cmd, queue_size=10,
        )

        # --- Publishers ---
        self._traj_pub = rospy.Publisher(
            "/uav/trajectory", PathMsg, queue_size=1, latch=True,
        )
        self._status_pub = rospy.Publisher(
            "/uav/planner_status", String, queue_size=10, latch=True,
        )

        rospy.loginfo("[planner] PlannerNode initialized")

    # ------------------------------------------------------------------
    # Importlib loader
    # ------------------------------------------------------------------
    @staticmethod
    def _load_planner(cls_path: str, kwargs: dict) -> Planner:
        module_path, class_name = cls_path.rsplit(".", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)
        return cls(**kwargs)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_pose(self, msg: PoseStamped) -> None:
        self._current_pose = msg

    def _on_cmd(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if cmd in ("LAND", "RTL", "EMERGENCY"):
            self._mission_ok = False
            rospy.logwarn("[planner] Mission paused (cmd=%s)", cmd)
        elif cmd in ("TAKEOFF", "GOTO", "HOVER"):
            self._mission_ok = True

    def _on_target(self, msg: PointStamped) -> None:
        if not self._mission_ok:
            rospy.loginfo("[planner] Skipping — mission not active")
            return

        if not self.planner.is_ready():
            self._publish_status("MAP_NOT_READY")
            rospy.logwarn_throttle(5.0, "[planner] Planner not ready")
            return

        self._publish_status("PLANNING")

        start = self._current_pose.pose.position
        goal = msg.point

        traj = self.planner.plan(start, goal)
        if traj is None:
            self._publish_status("FAIL")
            rospy.logerr("[planner] Planning failed")
            return

        path_msg = self._trajectory_to_path(traj)
        self._traj_pub.publish(path_msg)
        self._publish_status("OK")
        rospy.loginfo(
            "[planner] Trajectory published: %d points, %.2f s",
            len(traj.points),
            traj.points[-1].t_from_start if traj.points else 0.0,
        )

    # ------------------------------------------------------------------
    # Trajectory → nav_msgs/Path conversion
    # ------------------------------------------------------------------
    @staticmethod
    def _trajectory_to_path(traj: Trajectory) -> PathMsg:
        """Convert Trajectory to nav_msgs/Path.

        t_from_start is stored in PoseStamped.header.stamp as a Duration
        so the follower can reconstruct timing by subtracting Time(0).
        """
        path = PathMsg()
        path.header = Header(frame_id=traj.frame_id)

        for pt in traj.points:
            pose = PoseStamped()
            pose.header = Header(
                stamp=rospy.Time(secs=pt.t_from_start),
                frame_id=traj.frame_id,
            )
            pose.pose.position = Point(x=pt.pos[0], y=pt.pos[1], z=pt.pos[2])
            pose.pose.orientation = Quaternion(w=1.0)
            path.poses.append(pose)

        return path

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_status(self, status: str) -> None:
        self._status_msg.data = status
        self._status_pub.publish(self._status_msg)


# ==============================================================================
if __name__ == "__main__":
    node = PlannerNode()
    rospy.spin()
