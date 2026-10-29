import math
import unittest
from _support import ROOT
from navigation_core.control import Reference,ControlArbiter
from test_trajectory_execution import trajectory
class OwnershipTests(unittest.TestCase):
    def setUp(self):
        self.a=ControlArbiter(); self.anchor=Reference((0,0,2),math.pi)
        self.a.begin("a",1,"e",self.anchor)
    def test_hold_is_latched_and_resume_new_trajectory(self):
        self.assertTrue(self.a.track(trajectory(),100))
        self.a.hold(self.anchor); self.a.hold(Reference((4,4,2),0))
        self.assertEqual(self.a.output(100),self.anchor)
        self.assertTrue(self.a.track(trajectory(2,101),101))
        self.assertEqual(self.a.output(101.5).position,(5,0,2))
    def test_cancel_and_safety_dominate_late_output(self):
        self.a.track(trajectory(),100); self.a.cancel(self.anchor)
        self.assertFalse(self.a.track(trajectory(2,100),100))
        self.assertEqual(self.a.output(101),self.anchor)
        self.a.begin("a",2,"e",self.anchor); self.a.stop(self.anchor)
        self.assertFalse(self.a.track(trajectory(3,100,generation=2),100))
        with self.assertRaises(ValueError): self.a.begin("b",1,"e",self.anchor)
        self.a.clear_safety(); self.a.begin("b",1,"e",self.anchor)
    def test_no_pause_means_no_competing_hold(self):
        self.a.track(trajectory(),100)
        self.a.pause_detection(self.anchor,pause=False)
        self.assertEqual(self.a.mode,"TRACK")
        self.assertEqual(self.a.output(100.5).position,(5,0,2))
