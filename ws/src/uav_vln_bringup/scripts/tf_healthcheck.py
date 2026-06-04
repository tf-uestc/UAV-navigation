#!/usr/bin/env python3
"""tf_healthcheck.py — 启动时验证 TF 树完整性."""

import sys
import rospy
import tf2_ros


REQUIRED_TRANSFORMS = [
    ("map", "base_link"),
    ("base_link", "camera_link_optical"),
    ("map", "camera_link_optical"),
]
TIMEOUT_S = 10.0


def main() -> None:
    rospy.init_node("tf_healthcheck", anonymous=False)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("[tf_healthcheck] Waiting for TF tree...")
    deadline = rospy.Time.now() + rospy.Duration(TIMEOUT_S)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        all_ok = True
        for parent, child in REQUIRED_TRANSFORMS:
            if tf_buffer.can_transform(parent, child, rospy.Time(0)):
                rospy.loginfo("[tf_healthcheck]  ✓ %s → %s", parent, child)
            else:
                all_ok = False
                break

        if all_ok:
            rospy.loginfo("[tf_healthcheck] TF tree OK — all %d transforms ready",
                          len(REQUIRED_TRANSFORMS))
            rospy.spin()  # stay alive so required="true" doesn't kill launch

        if rospy.Time.now() > deadline:
            missing = [
                f"{p}→{c}"
                for p, c in REQUIRED_TRANSFORMS
                if not tf_buffer.can_transform(p, c, rospy.Time(0))
            ]
            rospy.logerr("[tf_healthcheck] TF timeout after %.0fs. Missing: %s",
                         TIMEOUT_S, ", ".join(missing))
            sys.exit(1)

        rate.sleep()


if __name__ == "__main__":
    main()
