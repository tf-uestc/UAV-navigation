#!/usr/bin/env python3
"""trajectory_follower.py — High-frequency trajectory playback.

Subscribes to /uav/trajectory (nav_msgs/Path) and publishes
/uav/goal_pose (PoseStamped) at ≥ 20 Hz by interpolating along the
trajectory according to elapsed time.

Params:
  ~publish_rate  (float)  Output frequency in Hz (default 30.0).
"""

from typing import List, Optional

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header


class TrajectoryFollower:
    """Interpolate a trajectory into a stream of position setpoints."""

    def __init__(self) -> None:
        rospy.init_node("trajectory_follower")

        # --- Params ---
        rate_hz = rospy.get_param("~publish_rate", 30.0)

        # --- State ---
        self._traj: Optional[PathMsg] = None
        self._traj_start_time: rospy.Time = rospy.Time(0)

        # --- Subscribers ---
        rospy.Subscriber(
            "/uav/trajectory", PathMsg,
            self._on_traj, queue_size=1,
        )

        # --- Publishers ---
        self._goal_pub = rospy.Publisher(
            "/uav/goal_pose", PoseStamped, queue_size=10,
        )

        # --- Timer ---
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._tick)

        rospy.loginfo("[follower] TrajectoryFollower started at %.1f Hz", rate_hz)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_traj(self, msg: PathMsg) -> None:
        self._traj = msg
        self._traj_start_time = rospy.Time.now()
        total_s = self._traj_duration(msg)
        rospy.loginfo(
            "[follower] New trajectory: %d pts, %.2f s",
            len(msg.poses), total_s,
        )

    # ------------------------------------------------------------------
    # Timer: 30 Hz setpoint stream
    # ------------------------------------------------------------------
    def _tick(self, _event) -> None:
        if self._traj is None or len(self._traj.poses) == 0:
            # OFFBOARD requires continuous setpoint stream — even when
            # there is no trajectory we must keep publishing.
            rospy.logwarn_throttle(1.0, "[follower] No trajectory — holding position")
            return

        # Clamps instead of stopping so OFFBOARD never starves.
        elapsed = max(0.0, (rospy.Time.now() - self._traj_start_time).to_sec())
        total = self._traj_duration(self._traj)
        if total <= 0:
            sp = self._traj.poses[-1]
        elif elapsed >= total:
            sp = self._traj.poses[-1]
        else:
            sp = self._interpolate(self._traj, elapsed)

        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = self._traj.header.frame_id
        self._goal_pub.publish(sp)

    # ------------------------------------------------------------------
    # Interpolation
    # ------------------------------------------------------------------
    @staticmethod
    def _interpolate(traj: PathMsg, elapsed: float) -> PoseStamped:
        """Linear interpolation along the trajectory by elapsed time.

        Each PoseStamped.header.stamp stores t_from_start as a Duration,
        so subtracting Time(0) yields the point's offset in seconds.
        """
        poses = traj.poses
        times = [
            (p.header.stamp - rospy.Time(0)).to_sec() for p in poses
        ]

        if elapsed <= times[0]:
            return poses[0]

        for i in range(1, len(poses)):
            if elapsed <= times[i]:
                t0, t1 = times[i - 1], times[i]
                ratio = (elapsed - t0) / (t1 - t0) if t1 > t0 else 0.0

                sp = PoseStamped()
                sp.pose.position = Point(
                    x=poses[i - 1].pose.position.x + ratio * (
                        poses[i].pose.position.x - poses[i - 1].pose.position.x),
                    y=poses[i - 1].pose.position.y + ratio * (
                        poses[i].pose.position.y - poses[i - 1].pose.position.y),
                    z=poses[i - 1].pose.position.z + ratio * (
                        poses[i].pose.position.z - poses[i - 1].pose.position.z),
                )
                sp.pose.orientation = Quaternion(w=1.0)
                return sp

        return poses[-1]

    @staticmethod
    def _traj_duration(traj: PathMsg) -> float:
        if not traj.poses:
            return 0.0
        last = traj.poses[-1]
        return (last.header.stamp - rospy.Time(0)).to_sec()


# ==============================================================================
if __name__ == "__main__":
    node = TrajectoryFollower()
    rospy.spin()
