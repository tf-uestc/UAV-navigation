import json
import unittest
from unittest.mock import patch
from _support import load_node,IdleThread,Time,Point,PoseStamped,PointStamped,Header,PathMessage,String
from navigation_core.contracts import RequestKey,DetectionResult

class MissionNodeTests(unittest.TestCase):
    def setUp(self):
        self.module=load_node("mission_manager")
        self.threads=patch("threading.Thread",IdleThread); self.threads.start()
        self.addCleanup(self.threads.stop)
        Time.current=100
        self.wall=0.
        self.node=self.module.MissionManager()
        self.node._clock=lambda:self.wall
        self.pose(0,0,2)
    def pose(self,x,y,z):
        p=PoseStamped(header=Header(stamp=Time.now(),frame_id="map")); p.pose.position=Point(x,y,z)
        self.node._on_pose(p)
    def start(self,target="red"):
        self.node._on_instruction(String(json.dumps({"target":{"text":target}})))
        self.node._trigger_vlm()
        request=json.loads(self.node._instruction_pub.messages[-1].data)
        return RequestKey(request["mission_id"],request["request_id"])
    def send(self,key,target="red",status="SUCCEEDED"):
        r=DetectionResult(key,target,status,99,100,(10,0,0) if status=="SUCCEEDED" else None)
        self.node._on_detection_result(String(r.to_json()))
    def approach(self):
        key=self.start(); self.send(key)
        self.assertEqual(self.node._state,"APPROACHING")
        self.assertEqual(self.node._approach_phase,"WAIT_PLAN")
    def test_legacy_target_cannot_hijack_and_old_task_rejected(self):
        old=self.start("red"); new=self.start("blue")
        self.node._on_target_world(PointStamped(point=Point(99,0,2)))
        self.send(old)
        self.assertEqual(self.node._state,"SEARCHING")
        self.send(new,"blue")
        self.assertEqual(self.node._state,"APPROACHING")
        self.assertEqual(self.node._last_target.point.z,0)
        self.assertEqual(self.node._approach_target_pub.messages[-1].point.x,8)
        self.assertEqual(self.node._approach_target_pub.messages[-1].point.z,2)
    def test_fast_busy_does_not_wait_whole_timeout(self):
        self.approach()
        self.node._approach_phase="FLYING"
        self.wall=6.; self.pose(0,0,2)
        def immediate(message):
            d=json.loads(message.data)
            r=DetectionResult(RequestKey(d["mission_id"],d["request_id"]),"red","BUSY",0,100)
            self.node._on_detection_result(String(r.to_json()))
        self.node._instruction_pub.on_publish=immediate
        self.node._on_housekeeping_tick(None)
        self.assertEqual(self.node._miss_count,1)
        self.assertEqual(self.node._approach_phase,"WAIT_PLAN")
        self.assertIsNone(self.node._requests.pending)
    def test_not_ready_exits_after_bounded_wait(self):
        self.approach(); self.node._on_planner_status(String("MAP_NOT_READY"))
        for instant in (12,24,36):
            self.wall=instant
            self.node._on_housekeeping_tick(None)
        self.assertEqual(self.node._state,"HOVER")
        self.assertTrue(self.node._cancel_pub.messages)
    def test_hover_clears_and_latches_heading_position(self):
        self.approach()
        self.node._on_state_cmd(String("HOVER"))
        self.assertEqual(len(self.node._traj_pub.messages[-1].poses),0)
        anchor=self.node._hover_goal
        self.wall=1; self.pose(1,1,2); self.node._on_housekeeping_tick(None)
        self.assertEqual(self.node._hover_goal,anchor)
        self.assertEqual(self.node._goal_pub.messages[-1].pose.position.x,anchor[0])
    def test_no_pause_does_not_publish_competing_hold(self):
        self.approach()
        self.node._pause_for_vlm=False; self.node._approach_phase="FLYING"
        self.wall=6.; self.pose(0,0,2)
        self.node._on_housekeeping_tick(None)
        before=len(self.node._goal_pub.messages)
        self.wall=7.; self.node._on_housekeeping_tick(None)
        self.assertEqual(len(self.node._goal_pub.messages),before)
        self.assertEqual(self.node._approach_phase,"WAIT_VLM")
    def test_arrival_needs_fresh_speed_and_dwell(self):
        self.approach()
        self.node._approach_phase="FLYING"
        for instant in (1,2.1):
            self.wall=instant; self.pose(8,0,2)
            import types
            self.node._on_velocity(types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=Point()))))
            self.node._on_housekeeping_tick(None)
        self.assertEqual(self.node._state,"HOVER")


    def test_result_after_deadline_consumes_timeout_once(self):
        self.approach()
        self.node._approach_phase="FLYING"
        self.wall=6.; self.pose(0,0,2); self.node._on_housekeeping_tick(None)
        key=self.node._requests.pending.key
        self.wall=27.; self.pose(0,0,2)
        self.send(key)
        self.assertFalse(self.node._detection_in_flight)
        self.assertIsNone(self.node._requests.pending)
        self.assertEqual(self.node._miss_count,1)

    def test_ros_clock_reset_cancels_task(self):
        self.approach()
        self.node._on_housekeeping_tick(None)
        Time.current=1.
        self.wall=1.; self.node._on_housekeeping_tick(None)
        self.assertEqual(self.node._state,"HOVER")
        self.assertIsNone(self.node._requests.pending)

    def test_small_refinement_keeps_live_path_when_not_paused(self):
        self.approach()
        self.node._pause_for_vlm=False; self.node._approach_phase="FLYING"
        self.wall=6.; self.pose(0,0,2)
        self.node._on_housekeeping_tick(None)
        key=self.node._requests.pending.key
        before=len(self.node._approach_target_pub.messages)
        result=DetectionResult(key,"red","SUCCEEDED",100,100,(10.05,0,0))
        self.node._on_detection_result(String(result.to_json()))
        self.assertEqual(self.node._approach_phase,"FLYING")
        self.assertEqual(len(self.node._approach_target_pub.messages),before)
        self.assertEqual(self.node._last_target.point.x,10.05)
        self.assertEqual(self.node._navigation_goal,(8,0,2))

    def test_invalid_pose_does_not_replace_valid_anchor(self):
        good=self.node._current_pose
        for frame,qw in (("camera",1),("map",0),("map",float("nan"))):
            bad=PoseStamped(header=Header(stamp=Time.now(),frame_id=frame))
            bad.pose.position=Point(7,8,9); bad.pose.orientation.w=qw
            self.node._on_pose(bad)
            self.assertIs(self.node._current_pose,good)
            self.assertIsNone(self.node._pose_mono)
