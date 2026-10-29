import types
import sys
import unittest
from unittest.mock import patch
from _support import load_node,ros_modules,IdleThread,Time,Point,PoseStamped,PointStamped,Header,String
class PlannerNodeTests(unittest.TestCase):
    def make(self):
        m=load_node("planner_node")
        with patch.dict(sys.modules,ros_modules()):
            n=m.PlannerNode()
        n._on_pose(PoseStamped(header=Header(stamp=Time(100),frame_id="map")))
        return m,n
    def test_cancelled_job_cannot_publish(self):
        m,n=self.make()
        class Planner:
            def plan(self,start,goal):
                n._on_cancel(String("stop"))
                return m.Trajectory(points=[types.SimpleNamespace(pos=(1,0,2),yaw=0,t_from_start=0)])
            def cancel(self): pass
        n.planner=Planner(); n._planning=True
        n._run_planning(PointStamped(point=Point(10,0,2)),n._job_generation)
        self.assertFalse(n._traj_pub.messages)
        self.assertFalse(n._planning)
    def test_latest_pending_survives_old_failure(self):
        m,n=self.make()
        new=PointStamped(point=Point(20,0,2))
        class Planner:
            def is_ready(self): return True
            def cancel(self): pass
            def plan(self,start,goal):
                n._on_target(new)
                return None
        n.planner=Planner(); n._planning=True
        with patch("threading.Thread",IdleThread):
            n._run_planning(PointStamped(point=Point(10,0,2)),n._job_generation)
        self.assertTrue(n._planning)
        self.assertIsNone(n._pending_target)
        self.assertFalse(n._traj_pub.messages)
