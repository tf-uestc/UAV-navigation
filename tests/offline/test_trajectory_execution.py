import copy
import math
import unittest
from _support import ROOT
from navigation_core.trajectory import Sample,TimedTrajectory,TrajectoryBuffer
def trajectory(identity=1,start=100,mission="a",epoch="e",generation=1):
    return TimedTrajectory(mission,generation,epoch,identity,start,
        (Sample((0,0,2),(1,0,0),math.pi,0),Sample((10,0,2),(0,0,0),math.pi,1)))
class TrajectoryTests(unittest.TestCase):
    def test_immutable_endpoint_hold(self):
        t=trajectory(); before=copy.deepcopy(t); b=TrajectoryBuffer(); b.begin("a",1,"e")
        self.assertTrue(b.accept(t,100)); self.assertEqual(b.sample(100).position,(0,0,2))
        for i in range(100):
            p=b.sample(101.1+i*.1); self.assertEqual(p.position,(10,0,2)); self.assertEqual(p.yaw,math.pi)
        self.assertEqual(t,before)
    def test_future_switch_and_cancel(self):
        b=TrajectoryBuffer(); b.begin("a",1,"e")
        self.assertTrue(b.accept(trajectory(),100))
        self.assertTrue(b.accept(trajectory(2,102),100.5))
        self.assertEqual(b.sample(101.5).position,(10,0,2))
        self.assertEqual(b.sample(102).position,(0,0,2))
        self.assertFalse(b.accept(trajectory(1,102),102))
        self.assertFalse(b.accept(trajectory(3,102,mission="b"),102))
        b.cancel(); self.assertFalse(b.accept(trajectory(4,102),102))
    def test_late_and_reset(self):
        b=TrajectoryBuffer(); b.begin("a",1,"e")
        self.assertTrue(b.accept(trajectory(),100.5)); self.assertEqual(b.sample(100.5).position,(5,0,2))
        self.assertIsNone(b.sample(1)); self.assertEqual(b.reason,"clock_reset")
        self.assertFalse(b.accept(trajectory(2,1),1))
        b.begin("a",2,"restarted")
        self.assertTrue(b.accept(trajectory(1,1,epoch="restarted",generation=2),1))
    def test_expired_and_bad_times(self):
        b=TrajectoryBuffer(); b.begin("a",1,"e")
        self.assertFalse(b.accept(trajectory(),102))
        with self.assertRaises(ValueError):
            TimedTrajectory("a",1,"e",1,0,(Sample((0,0,0),(0,0,0),0,1),))
