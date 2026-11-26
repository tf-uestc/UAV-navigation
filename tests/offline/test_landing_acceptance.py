"""Test the SITL driver's terminal-state checks without starting ROS."""
import importlib.util
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from _support import ROOT, ros_modules


def trial_type():
    modules = ros_modules()
    for name, attrs in {
        "rosgraph": (), "gazebo_msgs.srv": ("GetLinkState", "SpawnModel"),
        "mavros_msgs.msg": ("ExtendedState", "PositionTarget", "State"),
        "validate_flight_execution": ("FakeFcu", "decode"),
    }.items():
        modules[name] = ModuleType(name)
        for attr in attrs:
            setattr(modules[name], attr, object)
    modules["geometry_msgs.msg"].Pose = object
    modules["sensor_msgs"].point_cloud2 = object
    modules["sensor_msgs.msg"].PointCloud2 = object
    spec = importlib.util.spec_from_file_location("landing_trial", ROOT/"tools/deployment/validate_segments.py")
    node = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(node)
    return node.Trial


class LandingAcceptance(unittest.TestCase):
    def setUp(self):
        self.trial = object.__new__(trial_type())
        self.trial.trace = []
        self.trial.send = Mock()
        self.trial.flight = dict(flight_session_id="f", seq=7, mode="HOLD", owns_flight=True)
        self.trial.mission = dict(mission_session_id="m", mission_id="test", phase="FAILED", reason="target_not_found")
        self.trial.state = SimpleNamespace(connected=True, armed=False)
        self.trial.landed = SimpleNamespace(landed_state=1)

    def test_failed_business_mission_can_finish_cleanup_with_its_own_acknowledgements(self):
        t = self.trial
        def wait(predicate, timeout, message):
            t.flight = dict(t.flight, mode="COMPLETE", owns_flight=False)
            self.assertFalse(predicate())
            t.flight = dict(t.flight, seq=8)
            self.assertFalse(predicate())
            t.mission = dict(t.mission, phase="LANDED", mission_id="retired")
            self.assertFalse(predicate())
            t.mission = dict(t.mission, mission_id="test")
            t.state.armed = True
            self.assertFalse(predicate())
            t.state.armed = False
            self.assertTrue(predicate())
            self.assertEqual(timeout, 40)
        t.wait = wait
        t.stop_and_land()
        t.send.assert_called_once_with("LAND")
        self.assertEqual(t.trace[0][2]["prior_reason"], "target_not_found")

    def test_physical_landing_does_not_hide_executor_failure(self):
        t = self.trial
        def wait(predicate, timeout, message):
            t.flight = dict(t.flight, owns_flight=False, mode="FAILED", reason="landing_timeout", seq=8)
            predicate()
        t.wait = wait
        with self.assertRaisesRegex(AssertionError, "landing_timeout"):
            t.stop_and_land()

    def test_idle_fixture_requires_no_landing_mission(self):
        t = self.trial
        t.flight = dict(t.flight, owns_flight=False, mode="IDLE")
        t.mission = dict(t.mission, mission_id=None, phase="IDLE")
        t.stop_and_land()
        t.send.assert_not_called()
