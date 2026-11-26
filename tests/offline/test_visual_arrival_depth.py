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

    def acquisition_reply(self, found):
        n = self.node
        Time.current += 1.
        now = time.monotonic()
        n.odom = ((0., 0., 1.6), (0., 0., 0.), 0., 0., Time.current, now)
        n.flight = (dict(flight_session_id="flight", owns_flight=True, mode="HOLD",
                         context=None, stop_id=n.stop_id, stop_confirmed=True), now)
        if n.budget.calls == 0:
            n.settled_at = Time.current - .2
            n.request_detection(now)
        else:
            n.tick(now)
        self.assertEqual(n.phase, "WAIT_VLM")
        key = n.registry.pending.key
        result = self.module.DetectionResult(key, n.target, "SUCCEEDED" if found else "FAILED",
                                             Time.current, Time.current,
                                             (3.429, 0., .6) if found else None,
                                             reason="" if found else "target_not_found")
        n.on_detection(String(result.to_json()))

    def test_one_missed_confirmation_keeps_view_and_checks_again(self):
        n = self.node
        n.started = time.monotonic()
        n.budget.begin(n.started)
        self.acquisition_reply(True)
        self.assertEqual(n.phase, "CONFIRM_TARGET")
        self.acquisition_reply(False)
        self.assertEqual(n.phase, "RETRY_DETECT")
        self.assertFalse(n.flight_pub.messages)
        self.acquisition_reply(True)
        self.assertEqual((n.phase, n.budget.calls), ("AUTHORIZE", 3))

    def test_second_missed_confirmation_clears_target_and_searches(self):
        n = self.node
        n.started = time.monotonic()
        n.budget.begin(n.started)
        self.acquisition_reply(True)
        self.acquisition_reply(False)
        self.acquisition_reply(False)
        self.assertEqual((n.phase, n.after_stop), ("BRAKE", ("SCAN_NEXT", "target_not_found")))
        self.assertIsNone(n.acquisition_point)
        self.assertEqual(n.budget.calls, 3)

    def test_first_explicit_miss_keeps_original_scan_policy(self):
        n = self.node
        n.started = time.monotonic()
        n.budget.begin(n.started)
        self.acquisition_reply(False)
        self.assertEqual((n.phase, n.after_stop), ("BRAKE", ("SCAN_NEXT", "target_not_found")))
        self.assertEqual(n.budget.calls, 1)

    def test_color_conflict_rechecks_once_then_advances_scan_under_original_budget(self):
        n = self.node
        n.mode = "visual"
        n.phase = "SETTLE"
        n.after_stop = ("DETECT", "")
        n.settled_at = Time.current - 1.
        n.budget.begin(time.monotonic())
        n.started = time.monotonic()
        n.scan_base = 0.
        n.scan_index = 3
        n.intended_yaw = np.pi/2
        n.acquisition_point = (1., 2., 0.)
        n.flight = (dict(flight_session_id="flight", owns_flight=True, mode="HOLD",
                         context=None, stop_id=7, stop_confirmed=True), time.monotonic())
        n.request_detection(time.monotonic())

        def fresh_feedback():
            n.odom = ((0.,0.,1.6), (0.,0.,0.), np.pi/2, 0., Time.current, time.monotonic())
            n.flight = (n.flight[0], time.monotonic())

        for attempt in (1, 2):
            key = n.registry.pending.key
            result = self.module.DetectionResult(key,n.target,"FAILED",Time.current,Time.current,
                                                 reason="target_color_unconfirmed")
            n.on_detection(String(result.to_json()))
            self.assertIsNone(n.acquisition_point)
            if attempt == 1:
                self.assertEqual(n.phase,"RETRY_DETECT")
                self.assertEqual(len(n.flight_pub.messages),0)
                Time.current += 1.
                fresh_feedback()
                n.tick(time.monotonic())
                self.assertEqual(n.phase,"WAIT_VLM")
        self.assertEqual((n.phase,n.after_stop),("BRAKE",("SCAN_NEXT","target_color_unconfirmed")))
        n.flight[0]["stop_id"] = n.stop_id
        n.tick(time.monotonic())
        Time.current += .4
        fresh_feedback()
        n.tick(time.monotonic())
        self.assertEqual((n.phase,n.scan_index,n.budget.calls),("SCAN_TURN",4,2))
        self.assertEqual(len(n.plan_pub.messages),0)
        self.assertAlmostEqual(n.intended_yaw,-np.pi/2)

    def test_bad_depth_still_stops_without_scanning_or_planning(self):
        n = self.node
        n.budget.begin(time.monotonic())
        n.settled_at = Time.current
        n.request_detection(time.monotonic())
        failure = self.module.DetectionResult(n.registry.pending.key,n.target,"FAILED",
                        Time.current,Time.current,reason="invalid_target_depth")
        n.on_detection(String(failure.to_json()))
        self.assertEqual((n.phase,n.reason),("FAILED","invalid_target_depth"))
        self.assertEqual(len(n.plan_pub.messages),0)


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

    def test_empty_color_and_positive_box_require_semantic_confirmation(self):
        rgb, depth = self.scene(0)
        with self.assertRaisesRegex(self.module.DetectionRejected, "target_color_unconfirmed"):
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
