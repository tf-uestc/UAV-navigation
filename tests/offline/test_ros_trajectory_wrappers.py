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


    def test_search_heading_and_pause_do_not_bank_yaw_rate(self):
        m=load_node("trajectory_follower"); Time.current=100
        f=m.TrajectoryFollower()
        p=PoseStamped(); p.pose.orientation=Quaternion(*quaternion(0))
        f._on_pose(p)
        p.pose.orientation=Quaternion(*quaternion(math.pi)); f._on_pose(p)
        def path(start, desired):
            a=PoseStamped(header=Header(stamp=Time(start),frame_id="map"))
            b=PoseStamped(header=Header(stamp=Time(start+1),frame_id="map"))
            a.pose.orientation=b.pose.orientation=Quaternion(*quaternion(desired))
            return PathMessage(header=Header(stamp=Time(start),frame_id="map"),poses=[a,b])
        f._on_traj(path(100,math.pi)); Time.current=100.05; f._tick(None)
        q=f._goal_pub.messages[-1].pose.orientation
        self.assertAlmostEqual(abs(quaternion_yaw((q.x,q.y,q.z,q.w))),math.pi)
        f._on_traj(PathMessage()); Time.current=160; f._tick(None)
        f._on_traj(path(160,0)); Time.current=160.05; f._tick(None)
        q=f._goal_pub.messages[-1].pose.orientation
        self.assertLessEqual(abs(math.pi-quaternion_yaw((q.x,q.y,q.z,q.w))),.040000001)

    def test_cancelled_follower_rejects_late_path(self):
        from _support import String
        m=load_node("trajectory_follower"); Time.current=100
        f=m.TrajectoryFollower(); f._on_pose(PoseStamped())
        p=PoseStamped(header=Header(stamp=Time(100),frame_id="map"))
        path=PathMessage(header=Header(stamp=Time(100),frame_id="map"),poses=[p])
        f._on_cancel(String("cancel")); f._on_traj(path); f._tick(None)
        self.assertFalse(f._goal_pub.messages)
