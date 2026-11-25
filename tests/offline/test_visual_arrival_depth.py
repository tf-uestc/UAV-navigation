"""Regression cases for delayed visual arrival and sparse pole depth."""
import importlib.util
import sys
import time
import types
import unittest
from unittest.mock import patch

import numpy as np

from _support import SCRIPTS, String, Time, load_detector, ros_modules


class VisualArrivalTests(unittest.TestCase):
    def setUp(self):
        modules = ros_modules()
        modules["sensor_msgs"].point_cloud2 = types.SimpleNamespace()
        modules["sensor_msgs.msg"].PointCloud2 = object
        spec = importlib.util.spec_from_file_location("segment_under_test", SCRIPTS/"segment_mission.py")
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.node = self.module.SegmentMission()
        self.node.fence = dict(x_min=-10, x_max=10, y_min=-10, y_max=10, z_min=.3, z_max=3)
        self.node.target = "red pole"
        self.node.mission = "arrival"
        self.node.flight_session = "flight"
        self.node.stop_id = 7
        self.addCleanup(setattr, Time, "current", Time.current)
        Time.current = 20.

    def observe_twice(self, distance, speed=0., confirmed=True):
        n = self.node
        point = (distance, 0., .6)
        for _ in range(2):
            Time.current += 1.
            now = time.monotonic()
            n.odom = ((0., 0., 1.6), (speed, 0., 0.), 0., 0., Time.current, now)
            n.flight = (dict(flight_session_id="flight", owns_flight=True, mode="HOLD",
                             context=None, stop_id=7, stop_confirmed=confirmed), now)
            n.phase, n.settled_at = "WAIT_VLM", Time.current - .1
            n.registry.begin(n.mission)
            key = n.registry.issue(n.target, now, 12.)
            result = self.module.DetectionResult(key, n.target, "SUCCEEDED", Time.current,
                                                 Time.current, point)
            n.on_detection(String(result.to_json()))

    def test_too_close_stops_without_claiming_arrival(self):
        self.observe_twice(1.)
        self.assertEqual((self.node.phase, self.node.after_stop),
                         ("BRAKE", ("FAILED", "observation_too_close")))

    def test_moving_at_reply_requires_stop_and_new_observation(self):
        self.observe_twice(2.4, speed=.4)
        self.assertEqual((self.node.phase, self.node.after_stop[0] if self.node.after_stop else None),
                         ("BRAKE", "DETECT"))

    def test_missing_stop_confirmation_even_with_low_speed(self):
        self.observe_twice(2.4, confirmed=False)
        self.assertEqual(self.node.phase, "BRAKE")

    def test_inside_band_stopped(self):
        self.observe_twice(2.5)
        self.assertEqual(self.node.phase, "DONE")

    def test_outside_band_requires_plan(self):
        self.observe_twice(3.)
        self.assertEqual(self.node.phase, "AUTHORIZE")


class PoleDepthTests(unittest.TestCase):
    def setUp(self):
        self.module = load_detector("qwen_vl_grounding")
        self.detector = self.module.QwenVLGroundingDetector("test-only", coordinate_system="pixel",
                                                           response_protocol="canonical")
        self.detector._call_api = lambda *args: '{"found":true,"bbox":[0,0,100,100],"coordinate_system":"pixel"}'
        self.info = types.SimpleNamespace(K=[100,0,50,0,100,50,0,0,1])

    def scene(self, width):
        rgb = np.full((100, 100, 3), 160, np.uint8)
        depth = np.full((100, 100), 6., np.float32)
        rgb[:, 30:30+width] = (190,30,10)
        depth[:, 30:30+width] = 2.
        return rgb, depth

    def test_pole_surface_and_projection_in_loose_boxes(self):
        for width in (2, 8, 10, 20):
            with self.subTest(width=width):
                rgb, depth = self.scene(width)
                result = self.detector.detect(rgb, depth, "红色竖直杆", self.info)
                self.assertEqual(result.depth_m, 2.)
                self.assertAlmostEqual(result.point_camera[0], (30+(width-1)/2-50)*2/100)

    def test_target_pixels_without_depth_cannot_use_the_wall(self):
        rgb, depth = self.scene(8)
        depth[:, 30:38] = np.nan
        with self.assertRaisesRegex(self.module.DetectionRejected, "invalid_target_depth"):
            self.detector.detect(rgb, depth, "red pole", self.info)

    def test_partial_holes_and_unrelated_closer_surface(self):
        rgb, depth = self.scene(8)
        depth[:20, 30:38] = np.nan
        depth[:20, :20] = 1.
        result = self.detector.detect(rgb, depth, "red pole", self.info)
        self.assertEqual(result.depth_m, 2.)

    def test_generic_ambiguous_foreground_rejects_background(self):
        rgb, depth = self.scene(8)
        with self.assertRaisesRegex(self.module.DetectionRejected, "invalid_target_depth"):
            self.detector.detect(rgb, depth, "fixture", self.info)

    def test_ground_at_bottom_edge_does_not_hide_supported_foreground(self):
        rgb, depth = self.scene(20)
        for i in range(3):
            depth[-1-i] = .5*(i+1)
        result = self.detector.detect(rgb, depth, "fixture", self.info)
        self.assertEqual(result.depth_m, 2.)
