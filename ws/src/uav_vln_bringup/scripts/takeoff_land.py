#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class OffboardTakeoffLand:
    def __init__(self):
        self.state = State()
        self.current_pose = PoseStamped()

        self.target_alt = rospy.get_param("~takeoff_alt", 2.0)
        self.hover_time = rospy.get_param("~hover_time", 10.0)
        self.rate_hz = rospy.get_param("~setpoint_rate", 20.0)

        self.state_sub = rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=10)
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10
        )

        self.sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def _on_state(self, msg: State):
        self.state = msg

    def _on_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def wait_for_connection(self, timeout_s: float = 30.0):
        start = rospy.Time.now()
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.state.connected:
            if (rospy.Time.now() - start).to_sec() > timeout_s:
                raise RuntimeError("Timeout waiting for FCU connection via MAVROS")
            r.sleep()

    def _publish_setpoints_for(self, seconds: float, pose: PoseStamped):
        r = rospy.Rate(self.rate_hz)
        end_t = rospy.Time.now() + rospy.Duration.from_sec(seconds)
        while not rospy.is_shutdown() and rospy.Time.now() < end_t:
            pose.header.stamp = rospy.Time.now()
            self.sp_pub.publish(pose)
            r.sleep()

    def _set_mode(self, mode: str) -> bool:
        try:
            resp = self.set_mode_srv(custom_mode=mode)
            return bool(resp.mode_sent)
        except rospy.ServiceException:
            return False

    def _arm(self, value: bool) -> bool:
        try:
            resp = self.arming_srv(value)
            return bool(resp.success)
        except rospy.ServiceException:
            return False

    def run(self):
        rospy.loginfo("Waiting for FCU connection...")
        self.wait_for_connection()
        rospy.loginfo("FCU connected")

        # Use current x/y as takeoff point.
        x0 = self.current_pose.pose.position.x
        y0 = self.current_pose.pose.position.y

        sp = PoseStamped()
        sp.header.frame_id = "map"
        sp.pose.position.x = x0
        sp.pose.position.y = y0
        sp.pose.position.z = float(self.target_alt)
        sp.pose.orientation.w = 1.0

        # PX4 requires a stream of setpoints before enabling OFFBOARD.
        rospy.loginfo("Pre-publishing setpoints...")
        self._publish_setpoints_for(2.0, sp)

        rospy.loginfo("Arming...")
        if not self._arm(True):
            raise RuntimeError("Failed to arm")

        rospy.loginfo("Setting mode OFFBOARD...")
        if not self._set_mode("OFFBOARD"):
            raise RuntimeError("Failed to set mode OFFBOARD")

        # Takeoff + hover
        rospy.loginfo("Taking off to %.2fm", self.target_alt)
        self._publish_setpoints_for(5.0, sp)

        rospy.loginfo("Hovering for %.1fs", self.hover_time)
        self._publish_setpoints_for(self.hover_time, sp)

        # Land: hand back to PX4 land mode.
        rospy.loginfo("Switching to AUTO.LAND...")
        if not self._set_mode("AUTO.LAND"):
            rospy.logwarn("Failed to set AUTO.LAND, trying AUTO.RTL")
            self._set_mode("AUTO.RTL")

        # Keep publishing a bit to avoid sudden drop in offboard stream (not strictly needed after AUTO.LAND).
        self._publish_setpoints_for(2.0, sp)

        rospy.loginfo("Done")


def main():
    rospy.init_node("takeoff_land", anonymous=False)
    node = OffboardTakeoffLand()
    node.run()


if __name__ == "__main__":
    main()
