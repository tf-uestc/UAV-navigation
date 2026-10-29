import math
import unittest
from _support import ROOT
from navigation_core.bridge import BridgeDecisions,Feedback,reference_from_pose
from navigation_core.control import Reference
from navigation_core.yaw import quaternion
class BridgeTests(unittest.TestCase):
    def test_trajectory_not_shortened_and_explicit_resume(self):
        b=BridgeDecisions(); f=Feedback((0,0,2),math.pi,armed=True,mode="OFFBOARD",landed=False)
        b.command("HOVER",f,0)
        b.set_goal(Reference((.3,0,2),math.pi),0,f)
        self.assertEqual(b.mode,"HOLD")
        self.assertTrue(b.command("GOTO",f,0))
        self.assertEqual(b.tick(f,0).reference.position,(.3,0,2))
        out=b.tick(f,3)
        self.assertEqual(out.mode,"HOLD"); self.assertEqual(out.reference.position,(0,0,2))
    def test_hover_anchor_and_yaw_are_latched(self):
        b=BridgeDecisions(); f=Feedback((1,2,2),math.pi)
        b.command("HOVER",f,0)
        b.command("HOVER",Feedback((2,2,2),0),1)
        out=b.tick(Feedback((2,2,2),0),1)
        self.assertEqual(out.reference,Reference((1,2,2),math.pi))
    def test_takeoff_requires_feedback_not_service_sent(self):
        b=BridgeDecisions(); f=Feedback((0,0,0),math.pi)
        self.assertTrue(b.command("TAKEOFF",f,0))
        self.assertEqual(b.tick(f,1).actions,())
        self.assertIn("ARM",b.tick(f,2).actions)
        armed=Feedback((0,0,0),math.pi,armed=True)
        self.assertIn("SET_OFFBOARD",b.tick(armed,3).actions)
        self.assertEqual(b.tick(armed,4).mode,"WAIT_OFFBOARD")
        climbing=Feedback((0,0,.1),math.pi,armed=True,mode="OFFBOARD",landed=False)
        self.assertEqual(b.tick(climbing,5).mode,"CLIMB")
        done=Feedback((0,0,2),math.pi,armed=True,mode="OFFBOARD",landed=False)
        self.assertEqual(b.tick(done,6).mode,"HOLD")
    def test_cancel_preempts_each_takeoff_phase(self):
        for time in (0,2,4):
            b=BridgeDecisions(); f=Feedback((0,0,.2),math.pi,armed=True,landed=False)
            # Start on ground, then inject the airborne feedback.
            b.command("TAKEOFF",Feedback((0,0,0),math.pi),0)
            b.tick(f,time)
            b.command("LAND",f,time)
            out=b.tick(f,time+.01)
            self.assertEqual(out.mode,"LANDING")
            self.assertEqual(out.reference.position,(0,0,.2))
            self.assertNotEqual(out.mode,"CLIMB")
    def test_timeout_holds_actual_not_takeoff_target(self):
        b=BridgeDecisions(takeoff_timeout=5)
        b.command("TAKEOFF",Feedback((0,0,0),1),0)
        f=Feedback((0,0,.4),1,armed=True,mode="OFFBOARD",landed=False)
        out=b.tick(f,5)
        self.assertEqual(out.mode,"FAILED")
        self.assertEqual(out.reference.position,(0,0,.4))
    def test_no_airborne_takeoff_or_motor_stop(self):
        b=BridgeDecisions(); f=Feedback((0,0,2),1,armed=True,mode="OFFBOARD",landed=False)
        self.assertFalse(b.command("TAKEOFF",f,0))
        self.assertFalse(b.command("MOTOR_STOP",f,0))
    def test_landing_requires_landed_and_disarmed(self):
        b=BridgeDecisions(); flying=Feedback((0,0,1),0,armed=True,mode="OFFBOARD",landed=False)
        b.command("LAND",flying,0)
        self.assertIn("AUTO.LAND",b.tick(flying,0).actions)
        landing=Feedback((0,0,.1),0,armed=True,mode="AUTO.LAND",landed=False)
        self.assertEqual(b.tick(landing,1).mode,"LANDING")
        ground=Feedback((0,0,0),0,armed=True,mode="AUTO.LAND",landed=True)
        self.assertIn("DISARM",b.tick(ground,2).actions)
        self.assertEqual(b.tick(Feedback((0,0,0),0),3).mode,"COMPLETE")
    def test_invalid_frame_quaternion_stale_and_clock(self):
        for p,q,frame in [((float("nan"),0,0),quaternion(0),"map"),((0,0,0),(0,0,0,0),"map"),((0,0,0),quaternion(0),"camera")]:
            with self.assertRaises(ValueError): reference_from_pose(p,q,frame)
        b=BridgeDecisions()
        self.assertEqual(b.tick(Feedback((0,0,0),0,pose_age=10),10).mode,"FAILED")
        self.assertEqual(b.tick(Feedback((0,0,0),0),1).reason,"clock_reset")
    def test_direct_goal_offset_applied_once(self):
        b=BridgeDecisions(); f=Feedback((0,0,2),0,armed=True,mode="OFFBOARD",landed=False)
        b.command("HOVER",f,0)
        b.set_goal(Reference((10,0,2),0),0,f,kind="DIRECT_GOAL",standoff=1)
        b.command("GOTO",f,0)
        self.assertEqual(b.tick(Feedback((5,0,2),0,armed=True,mode="OFFBOARD",landed=False),1).reference.position,(9,0,2))
