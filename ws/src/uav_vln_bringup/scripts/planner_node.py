#!/usr/bin/env python3
"""planner_node.py — ROS planner node with importlib-based Planner loading.

Subscribes to the F2 target and drone pose, runs planning, publishes
trajectory as nav_msgs/Path and status as std_msgs/String.

Params:
  ~planner_class  (str)   Dotted path to Planner subclass (default
                           planning.dummy_planner.DummyPlanner).
  ~planner_args   (dict)  Keyword arguments forwarded to the planner
                           constructor.
  ~target_topic   (str)   Target topic to consume (default /uav/target_world).
"""

import importlib
import os
import sys
import threading

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header, String

# Ensure scripts/ is on PYTHONPATH
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from planning.base import Planner, Trajectory  # noqa: E402
from navigation_core.yaw import quaternion


class PlannerNode:
    """ROS node wrapping a Planner implementation."""

    def __init__(self) -> None:
        rospy.init_node("planner_node")
        self._status_msg = String()
        self._lock = threading.RLock()
        self._job_generation = 0
        self._pose_received = False

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
        self._planning: bool = False
        self._pending_target = None

        # --- Publishers ---
        self._traj_pub = rospy.Publisher(
            "/uav/trajectory", PathMsg, queue_size=1, latch=True,
        )
        self._status_pub = rospy.Publisher(
            "/uav/planner_status", String, queue_size=10, latch=True,
        )

        # --- Subscribers ---
        target_topic = rospy.get_param("~target_topic", "/uav/target_world")
        rospy.Subscriber(
            target_topic, PointStamped,
            self._on_target, queue_size=1,
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped,
            self._on_pose, queue_size=10,
        )
        rospy.Subscriber("/uav/cancel_planning", String, self._on_cancel, queue_size=10)
        rospy.Subscriber("/uav/mission_status", String, self._on_mission_state, queue_size=10)
        rospy.Subscriber(
            "/uav/state_cmd", String,
            self._on_cmd, queue_size=10,
        )

        rospy.loginfo("[planner] PlannerNode initialized (target_topic=%s)",
                      target_topic)

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
    def _on_pose(self, msg):
        from navigation_core.contracts import vector3
        try:
            vector3((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        except ValueError:
            return
        with self._lock:
            self._current_pose = msg
            self._pose_received = True

    def _on_cmd(self, msg):
        command = msg.data.strip().upper()
        if command in ("LAND", "RTL", "EMERGENCY", "HOVER"):
            self._on_cancel(msg)
        elif command in ("TAKEOFF", "GOTO", "RESUME"):
            with self._lock:
                self._mission_ok = True

    def _on_cancel(self, _msg):
        with self._lock:
            self._mission_ok = False
            self._job_generation += 1
            self._pending_target = None
            cancel = getattr(self.planner, "cancel", None)
            if cancel:
                cancel()
            self._publish_status("CANCELLED")

    def _on_mission_state(self, msg):
        if msg.data == "APPROACHING":
            with self._lock:
                self._mission_ok = True
        elif msg.data in ("IDLE", "SEARCHING", "HOVER"):
            self._on_cancel(msg)

    def _on_target(self, msg):
        with self._lock:
            if not self._mission_ok:
                return
            if not self._pose_received or not self.planner.is_ready():
                self._publish_status("MAP_NOT_READY")
                return
            self._job_generation += 1
            if self._planning:
                self._pending_target = msg
                cancel = getattr(self.planner, "cancel", None)
                if cancel:
                    cancel()
                return
            self._start_planning(msg)

    def _start_planning(self, msg):
        self._planning = True
        threading.Thread(target=self._run_planning,
                         args=(msg, self._job_generation), daemon=True).start()

    def _run_planning(self, msg, generation=None):
        if generation is None:
            generation = self._job_generation
        try:
            self._publish_status("PLANNING")
            start = self._current_pose.pose.position
            try:
                trajectory = self.planner.plan(start, msg.point)
                path = self._trajectory_to_path(trajectory) if trajectory and trajectory.points else None
            except Exception as exc:
                rospy.logerr("[planner] Planning failed: %s", exc)
                path = None
            with self._lock:
                if generation != self._job_generation or not self._mission_ok:
                    return
                if path is None:
                    self._publish_status("FAIL")
                else:
                    self._traj_pub.publish(path)
                    self._publish_status("OK")
        finally:
            with self._lock:
                self._planning = False
                if self._pending_target is not None and self._mission_ok:
                    latest = self._pending_target
                    self._pending_target = None
                    self._start_planning(latest)

    # ------------------------------------------------------------------
    # Trajectory → nav_msgs/Path conversion
    # ------------------------------------------------------------------
    @staticmethod
    def _trajectory_to_path(traj: Trajectory) -> PathMsg:
        """Convert Trajectory to nav_msgs/Path.

        Path header stores the original absolute start time; pose headers
        store absolute sample times. Never encode Duration as a ROS timestamp.
        """
        path = PathMsg()
        start = traj.stamp_at_start if traj.stamp_at_start is not None else rospy.Time.now().to_sec()
        path.header = Header(frame_id=traj.frame_id, stamp=rospy.Time.from_sec(start))

        for pt in traj.points:
            pose = PoseStamped()
            pose.header = Header(
                stamp=rospy.Time.from_sec(start + pt.t_from_start),
                frame_id=traj.frame_id,
            )
            pose.pose.position = Point(x=pt.pos[0], y=pt.pos[1], z=pt.pos[2])
            pose.pose.orientation = Quaternion(*quaternion(pt.yaw))
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
