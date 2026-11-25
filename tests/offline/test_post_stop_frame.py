import threading
import time
import unittest

import test_detector_node
from _support import Time
from navigation_core.contracts import DetectionResult


class NotifyingCondition(threading.Condition):
    """A barrier for testing the actual waiter without sleep-based synchronization."""
    def __init__(self):
        super().__init__()
        self.waiting = threading.Event()

    def wait(self, timeout=None):
        self.waiting.set()
        return super().wait(timeout)


class PostStopFrameTests(unittest.TestCase):
    def node(self):
        _,node=test_detector_node.DetectorNodeTests().make()
        node.lock=NotifyingCondition()
        node._synchronize_camera=True
        node._max_result_age=2.
        node._post_stop_frame_wait=.25
        node.camera_optical_frame='cam'
        node.latest_cam_info.header=node.latest_rgb.header
        node.tf_buffer.lookup_transform=lambda *a:object()
        calls=[]
        node.detector.detect=lambda *a:calls.append(a)
        return node,calls

    def test_cached_pre_stop_frame_waits_for_new_acquisition(self):
        from copy import deepcopy
        node,calls=self.node()
        worker=threading.Thread(target=node._run_detection,args=('red',1,'a',100.05))
        worker.start()
        if not node.lock.waiting.wait(.1):
            worker.join(.5)
            self.fail('Worker returned before an eligible synchronized frame arrived')
        self.assertFalse(calls)
        triplet=deepcopy((node.latest_rgb,node.latest_depth,node.latest_cam_info))
        for msg in triplet:
            msg.header.stamp=Time(100.06)
        node._cb_camera(*triplet)
        worker.join(.5)
        self.assertFalse(worker.is_alive())
        result=DetectionResult.from_json(node.pub_result.messages[0].data)
        self.assertEqual((result.key.mission_id,result.key.request_id,result.capture_stamp),('a',1,100.06))
        self.assertEqual(result.reason,'no_valid_detection')
        self.assertEqual(len(calls),1)

    def test_missing_triplet_wait_is_bounded_by_remaining_result_budget(self):
        node,calls=self.node()
        node.latest_rgb=node.latest_depth=node.latest_cam_info=None
        node._post_stop_frame_wait=2.
        node._max_result_age=.025
        started=time.monotonic()
        node._run_detection('red',1,'a',100.05)
        elapsed=time.monotonic()-started
        result=DetectionResult.from_json(node.pub_result.messages[0].data)
        self.assertEqual((result.reason,result.capture_stamp,result.key.request_id),
                         ('post_stop_frame_timeout',0.,1))
        self.assertGreaterEqual(elapsed,.02)
        self.assertLess(elapsed,.5)
        self.assertFalse(calls)
        self.assertFalse(node._detecting)
