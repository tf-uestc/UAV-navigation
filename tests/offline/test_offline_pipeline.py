"""Semantic pipeline replay with ideal state feedback, NOT a physics simulator."""
import json
import math
import sys
import types
import unittest
from unittest.mock import patch
from _support import (load_node,ros_modules,Time,Point,PoseStamped,Quaternion,Header,
                      PointStamped,PathMessage,String,IdleThread)
from navigation_core.contracts import DetectionResult,RequestKey
from navigation_core.yaw import quaternion

class Pipeline:
    def __init__(self):
        self.threads=patch("threading.Thread",IdleThread); self.threads.start()
        Time.current=100.; self.wall=0.; self.position=(0.,0.,2.)
        self.manager=load_node("mission_manager").MissionManager()
        self.manager._clock=lambda:self.wall
        self.follower=load_node("trajectory_follower").TrajectoryFollower()
        pmod=load_node("planner_node")
        with patch.dict(sys.modules,ros_modules()):
            self.planner=pmod.PlannerNode()
        self.request_messages=[]; self.outputs=[]; self.plan_enabled=True
        self.trace=[]
        class StraightPlanner:
            def is_ready(inner): return True
            def cancel(inner): pass
            def plan(inner,start,goal):
                # Deterministic sample oracle: physical tracking is intentionally absent.
                from planning.base import Trajectory,TrajectoryPoint
                distance=math.dist((start.x,start.y,start.z),(goal.x,goal.y,goal.z))
                duration=max(.1,distance/.5)
                return Trajectory(stamp_at_start=Time.current,points=[
                    TrajectoryPoint(pos=(start.x,start.y,start.z),yaw=0,t_from_start=0),
                    TrajectoryPoint(pos=(goal.x,goal.y,goal.z),yaw=0,t_from_start=duration)])
        self.planner.planner=StraightPlanner()
        self.manager._instruction_pub.on_publish=self.request
        self.manager._cancel_pub.on_publish=self.cancel
        self.manager._status_pub.on_publish=self.status
        self.manager._traj_pub.on_publish=self.follower._on_traj
        self.manager._approach_target_pub.on_publish=self.plan
        self.manager._goal_pub.on_publish=lambda m:self.output("mission",m)
        self.follower._goal_pub.on_publish=lambda m:self.output("follower",m)
        self.planner._traj_pub.on_publish=self.path
        self.planner._status_pub.on_publish=self.manager._on_planner_status
        self.feedback()
    def close(self): self.threads.stop()
    def request(self,message):
        data=json.loads(message.data); data["_capture_stamp"]=Time.current
        self.request_messages.append(data)
        self.trace.append(dict(event="request",mission_id=data["mission_id"],request_id=data["request_id"],
                               wall=self.wall,capture_stamp=Time.current))
    def output(self,owner,message):
        self.outputs.append((owner,self.wall,message))
        self.trace.append(dict(event="reference",owner=owner,mission_id=self.manager._mission_id,
            goal_generation=self.manager._plan_key,compat_sequence=self.follower._sequence,wall=self.wall,
            position=[message.pose.position.x,message.pose.position.y,message.pose.position.z]))
    def cancel(self,message):
        self.planner._on_cancel(message); self.follower._on_cancel(message)
    def status(self,message):
        self.planner._on_mission_state(message); self.follower._on_mission_state(message)
    def path(self,message):
        self.follower._on_traj(message); self.manager._on_trajectory(message)
    def plan(self,message):
        if self.plan_enabled:
            self.planner._on_target(message)
            if self.planner._planning:
                self.planner._run_planning(message,self.planner._job_generation)
    def feedback(self,velocity=(0.,0.,0.)):
        p=PoseStamped(header=Header(stamp=Time.now(),frame_id="map"))
        p.pose.position=Point(*self.position); p.pose.orientation=Quaternion(*quaternion(0))
        self.manager._on_pose(p); self.planner._on_pose(p); self.follower._on_pose(p)
        self.manager._on_velocity(types.SimpleNamespace(twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=Point(*velocity)))))
    def step(self,dt=.1,ideal_follow=True):
        previous=self.position
        self.wall+=dt; Time.current=100+self.wall
        if ideal_follow and self.outputs:
            p=self.outputs[-1][2].pose.position; self.position=(p.x,p.y,p.z)
        velocity=tuple((a-b)/dt for a,b in zip(self.position,previous))
        self.feedback(velocity)
        before=len(self.outputs)
        self.manager._on_housekeeping_tick(None); self.follower._tick(None)
        owners={item[0] for item in self.outputs[before:]}
        if len(owners)>1: raise AssertionError("two controllers emitted in one replay tick")
    def start(self,target="red"):
        self.manager._on_instruction(String(json.dumps({"target":{"text":target}})))
        self.manager._trigger_vlm()
        return self.request_messages[-1]
    def respond(self,request,status="SUCCEEDED",point=(3.,0.,0.)):
        capture=request["_capture_stamp"]
        self.trace.append(dict(event="result",mission_id=request["mission_id"],request_id=request["request_id"],status=status,wall=self.wall))
        r=DetectionResult(RequestKey(request["mission_id"],request["request_id"]),
            request["target"]["text"],status,capture,Time.current,
            point if status=="SUCCEEDED" else None)
        self.manager._on_detection_result(String(r.to_json()))

class PipelineReplayTests(unittest.TestCase):
    def setUp(self):
        self.h=Pipeline(); self.addCleanup(self.h.close)
    def test_normal_approach_and_ten_seconds_final_hold(self):
        h=self.h; h.respond(h.start())
        self.assertEqual(h.manager._approach_phase,"FLYING")
        for _ in range(140): h.step()
        self.assertEqual(h.manager._state,"HOVER")
        self.assertAlmostEqual(h.position[0],1.,places=5)
        self.assertFalse(h.manager._requests.pending)
    def test_pause_refine_resume(self):
        h=self.h; h.manager._approach_vlm_interval=.5
        h.respond(h.start())
        for _ in range(6): h.step()
        self.assertEqual(h.manager._approach_phase,"WAIT_VLM")
        anchor=h.outputs[-1][2].pose.position.x
        for _ in range(10): h.step()
        self.assertAlmostEqual(h.position[0],anchor)
        h.respond(h.request_messages[-1],point=(3.05,0.,0.))
        self.assertEqual(h.manager._approach_phase,"FLYING")
    def test_busy_recovers_once(self):
        h=self.h; h.respond(h.start())
        h.manager._approach_vlm_interval=.1
        h.step(.2)
        h.respond(h.request_messages[-1],"BUSY")
        self.assertEqual(h.manager._miss_count,1)
        self.assertEqual(h.manager._approach_phase,"FLYING")
    def test_api_timeout_and_late_result(self):
        h=self.h; h.respond(h.start()); h.manager._approach_vlm_interval=.1
        h.step(.2); request=h.request_messages[-1]
        h.step(21,ideal_follow=False)
        self.assertEqual(h.manager._miss_count,1)
        count=len(h.manager._approach_target_pub.messages)
        h.respond(request)
        self.assertEqual(len(h.manager._approach_target_pub.messages),count)
    def test_old_task_cannot_control_new_task(self):
        h=self.h; old=h.start("red"); new=h.start("blue")
        h.respond(old); self.assertEqual(h.manager._state,"SEARCHING")
        h.respond(new); self.assertEqual(h.manager._state,"APPROACHING")
    def test_cancel_drops_detector_and_late_path(self):
        h=self.h; request=h.start()
        h.manager._on_state_cmd(String("HOVER")); h.respond(request)
        p=PoseStamped(header=Header(stamp=Time.now(),frame_id="map")); p.pose.position=Point(99,0,2)
        h.follower._on_traj(PathMessage(header=Header(stamp=Time.now(),frame_id="map"),poses=[p]))
        h.step()
        self.assertEqual(h.manager._state,"HOVER")
        self.assertNotEqual(h.position[0],99)
    def test_plan_deadline(self):
        h=self.h; h.plan_enabled=False; h.respond(h.start())
        for _ in range(4): h.step(12,ideal_follow=False)
        self.assertEqual(h.manager._state,"HOVER")
    def test_target_jump_is_not_forwarded(self):
        h=self.h; h.respond(h.start()); h.manager._approach_vlm_interval=.1
        h.step(.2); count=len(h.manager._approach_target_pub.messages)
        h.respond(h.request_messages[-1],point=(15.,0.,0.))
        self.assertEqual(h.manager._target_estimator.reason,"jump")
        # A controlled retry may reuse the previous safe goal, never the jump.
        goals=h.manager._approach_target_pub.messages[count:]
        self.assertTrue(all(p.point.x<5 for p in goals))
    def test_ros_clock_reset_revokes_old_playback(self):
        h=self.h; h.respond(h.start()); h.step()
        Time.current=1.
        before=len(h.follower._goal_pub.messages)
        h.follower._tick(None)
        self.assertEqual(len(h.follower._goal_pub.messages),before)
        self.assertEqual(h.follower._buffer.reason,"clock_reset")
        h.manager._on_housekeeping_tick(None)
        self.assertEqual(h.manager._state,"HOVER")
        self.assertFalse(h.follower._enabled)


    def test_latency_cases_keep_correct_request(self):
        for latency in (.1,3.,10.):
            h=Pipeline()
            try:
                request=h.start()
                h.step(latency,ideal_follow=False)
                h.respond(request)
                self.assertEqual(h.manager._state,"APPROACHING")
                self.assertIsNone(h.manager._requests.pending)
            finally:
                h.close()

    def test_trace_connects_business_ids_and_owner(self):
        h=self.h; request=h.start(); h.respond(request); h.step()
        requests=[x for x in h.trace if x["event"]=="request"]
        results=[x for x in h.trace if x["event"]=="result"]
        outputs=[x for x in h.trace if x["event"]=="reference"]
        self.assertEqual(requests[0]["mission_id"],results[0]["mission_id"])
        self.assertEqual(requests[0]["request_id"],results[0]["request_id"])
        self.assertTrue(outputs)
        self.assertTrue(all(x["mission_id"]==request["mission_id"] for x in outputs))
