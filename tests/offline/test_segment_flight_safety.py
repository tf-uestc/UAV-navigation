"""Exercise the production segment and SITL executor callbacks without ROS transport."""
import importlib.util
import json
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

from _support import Header, IdleThread, Point, PoseStamped, SCRIPTS, String, Time, ros_modules


def load_components():
    modules = ros_modules()
    modules["sensor_msgs"].point_cloud2 = object()
    modules["sensor_msgs.msg"].PointCloud2 = object
    for name, classes in {
        "rosgraph": (), "gazebo_msgs.srv": ("GetWorldProperties",),
        "mavros_msgs.msg": ("ExtendedState", "State"), "mavros_msgs.srv": ("CommandBool", "SetMode"),
    }.items():
        module = ModuleType(name)
        for attr in classes:
            setattr(module, attr, object)
        modules[name] = module
    result = []
    with patch.dict(sys.modules, modules):
        for name in ("flight_executor", "segment_mission"):
            spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + ".py"))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            result.append(module)
    return result


class SafetyTests(unittest.TestCase):
    def setUp(self):
        executor, mission = load_components()
        self.wall = 20.
        self.old_stamp = Time.current
        self.addCleanup(setattr, Time, "current", self.old_stamp)
        Time.current = 20.
        clock = patch("time.monotonic", side_effect=lambda: self.wall)
        clock.start()
        self.addCleanup(clock.stop)
        with patch("threading.Thread", IdleThread), patch.object(executor.FlightExecutor, "require_sitl"):
            self.executor = executor.FlightExecutor()
        self.mission = mission.SegmentMission()
        self.mission.flight_session = self.executor.session
        self.executor.state = (SimpleNamespace(connected=True, armed=True, mode="OFFBOARD"), self.wall)

    def advance(self, seconds):
        self.wall += seconds
        Time.current += seconds

    def sensors(self, speed, ground=True, update_ground=True):
        if update_ground:
            landed = SimpleNamespace(header=Header(stamp=Time.now()), landed_state=1 if ground else 2)
            self.executor.capture(landed, "landed")
        stamp = Time.now()
        stamp.is_zero = lambda: False
        pose = PoseStamped()
        pose.pose.position.z = .4
        odom = SimpleNamespace(header=Header(stamp=stamp, frame_id="map"), child_frame_id="base_link",
                               pose=pose, twist=SimpleNamespace(twist=SimpleNamespace(linear=Point(0, 0, speed))))
        self.executor.capture(odom, "odom")
        self.executor.clock_advanced_at = self.wall
        return self.executor.feedback(self.wall, Time.current)

    def test_takeoff_cancel_ignores_old_ground_during_ascent(self):
        node = self.executor
        fb = self.sensors(0)
        self.advance(.1)
        fb = self.sensors(0)
        node.bridge.command("TAKEOFF", fb, self.wall)
        node.owned = True
        self.advance(1.1)
        fb = self.sensors(.35, update_ground=False)
        node.command(String(json.dumps(dict(command="LAND", seq=1, flight_session_id=node.session))))
        decision = node.bridge.tick(node.feedback(self.wall, Time.current), self.wall)
        self.assertNotIn("DISARM", decision.actions, decision)
        self.assertIn("AUTO.LAND", decision.actions, decision)

    def test_stationary_cancel_requires_post_request_ground_feedback(self):
        node = self.executor
        self.sensors(0)
        self.advance(.1)
        fb = self.sensors(0)
        node.owned = True
        node.command(String(json.dumps(dict(command="LAND", seq=1, flight_session_id=node.session))))
        fb = node.feedback(self.wall, Time.current)
        self.assertNotIn("DISARM", node.bridge.tick(fb, self.wall).actions)
        self.advance(.3)
        self.sensors(0)
        self.advance(.3)
        fb = self.sensors(0)
        self.assertIn("DISARM", node.bridge.tick(fb, self.wall).actions)

    def test_stale_track_status_sends_stop_before_terminal(self):
        mission = self.mission
        mission.phase = "TRACK"
        mission.mission = "trial"
        mission.goal_id = 1
        mission.epoch = ("gateway", "planner")
        mission.flight = (dict(flight_session_id=self.executor.session, seq=3), self.wall-1)
        mission.tick(self.wall)
        commands = [json.loads(m.data) for m in mission.flight_pub.messages]
        self.assertEqual([c["command"] for c in commands], ["HOVER"])
        self.assertNotEqual(mission.phase, "FAILED")
        self.advance(.3)
        mission.on_flight(String(json.dumps(dict(flight_session_id=self.executor.session, seq=mission.seq,
            owns_flight=True, mode="HOLD", context=None, stop_confirmed=True, stop_id=mission.stop_id))))
        mission.tick(self.wall)
        self.assertEqual((mission.phase, mission.reason), ("FAILED", "flight_feedback_unavailable"))

    def test_normal_landing_rejects_airborne_then_accepts_ground(self):
        node = self.executor
        node.owned = True
        self.sensors(.2, ground=False)
        node.command(String(json.dumps(dict(command="LAND", seq=1, flight_session_id=node.session))))
        self.advance(.2)
        fb = self.sensors(.25, ground=True)
        self.assertNotIn("DISARM", node.bridge.tick(fb, self.wall).actions)
        self.advance(.2)
        self.sensors(0)
        self.advance(.2)
        fb = self.sensors(0)
        self.assertIn("DISARM", node.bridge.tick(fb, self.wall).actions)

    def test_only_recent_round_trip_for_active_goal_keeps_plan_alive(self):
        from navigation_core.control import Reference
        node, mission = self.executor, self.mission
        node.owned = True
        self.sensors(0, ground=False)
        node.bridge.mode = "HOLD"
        node.bridge.anchor = Reference((0., 0., .4), 0)
        node.planner, node.planner_at = ("gateway", "planner"), self.wall
        def send(command, **fields):
            node.command(String(json.dumps(dict(command=command, flight_session_id=node.session, **fields))))
        send("BEGIN", seq=1, mission_id="trial", goal_id=1, gateway_session_id="gateway",
             planner_session_id="planner", require_status_ack=True)
        ctx = ["trial", 1, "planner"]
        self.assertEqual(list(node.control.buffer.context), ctx)
        self.advance(.4)
        mission.phase, mission.epoch, mission.mission, mission.goal_id = "WAIT_PLAN", ("gateway", "planner"), "trial", 1
        mission.started, mission.deadline = self.wall, self.wall+5.
        status = dict(flight_session_id=node.session, seq=1, mode="HOLD", owns_flight=True,
                      status_at=self.wall, context=ctx)
        mission.on_flight(String(json.dumps(status)))
        mission.tick(self.wall)
        ack = mission.flight_pub.messages[-1]
        node.command(ack)
        renewed = node.mission_ack_deadline
        self.assertEqual(renewed, self.wall+.7)
        self.advance(.35)
        node.command(ack)
        send("ACK_STATUS", status_at=self.wall, context=["old", 1, "planner"])
        self.assertEqual(node.mission_ack_deadline, renewed)
        node.planner_at = self.wall
        fb = self.sensors(0, ground=False)
        node.check_plan_authority(fb, self.wall)
        self.assertTrue(node.control.live)
        self.advance(.37)
        node.planner_at = self.wall
        send("ACK_STATUS", status_at=self.wall, context=ctx)
        node.check_plan_authority(self.sensors(0, ground=False), self.wall)
        self.assertFalse(node.control.live)
        self.assertIsNone(node.control.buffer.context)
        self.assertEqual((node.bridge.mode, node.stop_reason), ("HOLD", "mission_feedback_timeout"))
        cancel = json.loads(node.cancel_pub.messages[-1].data)
        self.assertEqual((cancel["mission_id"], cancel["through_goal_id"], cancel["gateway_session_id"]),
                         ("trial", 1, "gateway"))

    def test_missing_abort_ack_is_reported_as_unconfirmed(self):
        mission = self.mission
        mission.phase = "TRACK"
        mission.flight = (dict(flight_session_id=self.executor.session, seq=2), self.wall-1)
        mission.tick(self.wall)
        self.advance(mission.settle_timeout+.01)
        mission.tick(self.wall)
        self.assertEqual((mission.phase, mission.reason), ("FAILED","flight_feedback_unavailable_stop_unconfirmed"))
