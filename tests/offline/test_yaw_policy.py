import math
import unittest
from _support import ROOT
from navigation_core.yaw import YawPolicy,angle_delta,quaternion,quaternion_yaw
class YawTests(unittest.TestCase):
    def test_hold_pi(self):
        p=YawPolicy(math.pi)
        self.assertEqual(p.update(None,10),math.pi)
        self.assertEqual(p.face((1,1,1),(1,1,2),1),math.pi)
    def test_shortest_wrap_and_rate(self):
        p=YawPolicy(math.radians(179),.1)
        value=p.update(math.radians(-179),.1)
        self.assertGreater(value,math.radians(179))
        self.assertLessEqual(value-math.radians(179),.010000001)
        self.assertAlmostEqual(angle_delta(math.radians(-179),math.radians(179)),math.radians(2))
    def test_quaternion_sign_and_invalid(self):
        q=quaternion(2.9)
        self.assertAlmostEqual(quaternion_yaw(q),quaternion_yaw(tuple(-v for v in q)))
        for q in [(0,0,0,0),(0,0,0,2),(0,0,float("nan"),1)]:
            with self.assertRaises(ValueError): quaternion_yaw(q)
