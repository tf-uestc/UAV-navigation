"""Lifecycle test detector with an observable, bounded concurrent-request barrier."""
import time

import rospy
from test_backend import FixtureDetector


class BlockingFixture(FixtureDetector):
    def detect(self, rgb, depth, target_text, cam_info):
        if target_text == 'lifecycle_busy_barrier':
            rospy.set_param('/stage78/fixture_entered', True)
            deadline = time.monotonic() + 15.
            while rospy.get_param('/stage78/fixture_hold', False) and not rospy.is_shutdown():
                if time.monotonic() >= deadline:
                    raise RuntimeError('lifecycle BUSY fixture was not released')
                time.sleep(.03)
        return super().detect(rgb, depth, target_text, cam_info)
