import copy
import math
import types
import unittest
from _support import load_node,Time,Header,Point,Quaternion,PoseStamped,PathMessage
from navigation_core.yaw import quaternion,quaternion_yaw
class RosTrajectoryTests(unittest.TestCase):
    def test_adapter_real_spline_domain(self):
        adapter=load_node("planning/ego_adapter")
        msg=types.SimpleNamespace(order=3,start_time=Time(100),traj_id=8,
            pos_pts=[Point(x,0,2) for x in range(-1,6)],knots=[(i-3)*.6 for i in range(11)])
        t=adapter.EgoPlannerAdapter._bspline_to_trajectory(msg)
        self.assertAlmostEqual(t.points[0].pos[0],0)
        self.assertAlmostEqual(t.points[-1].pos[0],4)
        self.assertAlmostEqual(t.points[-1].t_from_start,2.4)
        self.assertEqual(t.stamp_at_start,100)
    def test_planner_preserves_yaw_and_absolute_time(self):
        planner=load_node("planner_node")
        from planning.base import Trajectory,TrajectoryPoint
        p=planner.PlannerNode._trajectory_to_path(Trajectory(
            points=[TrajectoryPoint(pos=(1,2,3),yaw=math.pi,t_from_start=0)],stamp_at_start=100))
        q=p.poses[0].pose.orientation
        self.assertAlmostEqual(abs(quaternion_yaw((q.x,q.y,q.z,q.w))),math.pi)
        self.assertEqual(p.header.stamp.to_sec(),100)
        self.assertEqual(p.poses[0].header.stamp.to_sec(),100)
    def test_follower_endpoint_holds_without_cache_mutation(self):
        m=load_node("trajectory_follower"); Time.current=100
        f=m.TrajectoryFollower()
        pose=PoseStamped(); pose.pose.orientation=Quaternion(*quaternion(math.pi))
        if hasattr(f,"_on_pose"): f._on_pose(pose)
        a=PoseStamped(header=Header(stamp=Time(100),frame_id="map"))
        b=PoseStamped(header=Header(stamp=Time(101),frame_id="map"))
        a.pose.orientation=b.pose.orientation=Quaternion(*quaternion(math.pi))
        b.pose.position=Point(10,0,0)
        path=PathMessage(header=Header(stamp=Time(100),frame_id="map"),poses=[a,b])
        before=copy.deepcopy(path)
        f._on_traj(path)
        for i in range(100):
            Time.current=101.1+i*.1
            f._tick(None)
            self.assertEqual(f._goal_pub.messages[-1].pose.position.x,10)
        self.assertEqual(path.poses[-1].header.stamp,before.poses[-1].header.stamp)
