import unittest
from _support import ROOT
from navigation_core.targets import TargetEstimator,ArrivalTracker,observation_goal,lawnmower
FENCE=dict(x_min=-20,x_max=20,y_min=-20,y_max=20,z_min=1,z_max=3)
class TargetTests(unittest.TestCase):
    def test_estimate_age_jump_and_deadband(self):
        e=TargetEstimator()
        self.assertTrue(e.observe((5,0,0),1,2)); self.assertTrue(e.needs_replan()); e.mark_sent()
        self.assertTrue(e.observe((5.05,0,0),2,3)); self.assertFalse(e.needs_replan())
        self.assertFalse(e.observe((15,0,0),3,4)); self.assertEqual(e.point,(5.05,0,0))
        self.assertFalse(e.observe((5,0,0),1,40))
        e.reset(); self.assertTrue(e.observe((15,0,0),40,40))
    def test_ground_object_and_standoff(self):
        point=(10,0,0)
        goal=observation_goal((0,0,2),point,2,2,FENCE)
        self.assertEqual(goal.position,(8,0,2)); self.assertEqual(point,(10,0,0))
        for target,height in [((21,0,0),2),((0,0,0),2),((10,0,0),10)]:
            with self.assertRaises(ValueError): observation_goal((0,0,2),target,height,2,FENCE)
    def test_arrival_speed_dwell_and_clock_reset(self):
        a=ArrivalTracker()
        self.assertFalse(a.update((8,0,2),(1,0,0),(8,0,2),0))
        self.assertFalse(a.update((8,0,2),(0,0,0),(8,0,2),1))
        self.assertTrue(a.update((8,0,2),(0,0,0),(8,0,2),2))
        self.assertFalse(a.update((8,0,2),(0,0,0),(8,0,2),0))
        self.assertFalse(a.update((10,0,2),(0,0,0),(10,0,2),1))
    def test_bounded_search_pattern(self):
        points=lawnmower((0,0),30,30,5,2)
        self.assertEqual(len(points),14); self.assertEqual(points[0],(-15,-15,2))
        self.assertTrue(all(-15<=p[0]<=15 and -15<=p[1]<=15 for p in points))
        with self.assertRaises(ValueError): lawnmower((0,0),30,30,.001,2)
