import unittest
from _support import ROOT
from navigation_core.bspline import BSpline
class BSplineTests(unittest.TestCase):
    def assertVector(self,a,b):
        for x,y in zip(a,b): self.assertAlmostEqual(x,y,places=8)
    def test_uniform_domain_and_velocity(self):
        s=BSpline([(x,0,0) for x in range(-1,6)],[(i-3)*.6 for i in range(11)],3)
        self.assertAlmostEqual(s.duration,2.4)
        for t in [0,.1,.6,1.2,2.4]:
            self.assertVector(s.evaluate(t),(t/.6,0,0))
            self.assertVector(s.derivative().evaluate(t),(1/.6,0,0))
        self.assertAlmostEqual(list(s.samples())[-1][0],2.4)
    def test_bezier_independent_polynomial(self):
        s=BSpline([(0,0,0),(1,2,0),(2,2,0),(3,0,0)],[0]*4+[1]*4,3)
        for i in range(101):
            t=i/100
            self.assertVector(s.evaluate(t),(3*t,6*t*(1-t),0))
            self.assertVector(s.derivative().evaluate(t),(3,6-12*t,0))
    def test_nonuniform(self):
        s=BSpline([(0,0,0),(1,0,0),(3,0,0)],[0,0,1,3,3],1)
        self.assertVector(s.evaluate(2),(2,0,0))
    def test_invalid(self):
        for args in [([],[],3),([(0,0,0)]*4,[0]*8,3),([(0,0,0)]*2,[0,1,0,2],1),
                     ([(float("nan"),0,0)]*4,[0]*4+[1]*4,3)]:
            with self.assertRaises(ValueError): BSpline(*args)
