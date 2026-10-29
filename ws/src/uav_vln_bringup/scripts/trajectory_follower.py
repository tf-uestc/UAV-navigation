#!/usr/bin/env python3
"""Play absolute-time Path samples without mutating received messages.

Path.header.stamp is trajectory start; each pose stamp is absolute sample time.
This compatibility transport has no EGO goal acknowledgement; U05/U06 must
validate the complete continuous trajectory/ownership interface before flight.
"""
import copy
import threading
import uuid
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header
from navigation_core.trajectory import Sample, TimedTrajectory, TrajectoryBuffer
from navigation_core.yaw import YawPolicy, quaternion, quaternion_yaw

class TrajectoryFollower:
    def __init__(self):
        rospy.init_node("trajectory_follower")
        rate = rospy.get_param("~publish_rate", 30.0)
        if rate <= 0:
            raise ValueError("publish_rate must be positive")
        self._lock = threading.RLock()
        self._buffer = TrajectoryBuffer()
        self._epoch = uuid.uuid4().hex
        self._generation = self._sequence = 0
        self._buffer.begin("path-compat", self._generation, self._epoch)
        self._yaw = None
        self._yaw_last_time = None
        self._yaw_rate = rospy.get_param("~yaw_rate_limit", .8)
        self._traj = None
        self._goal_pub = rospy.Publisher("/uav/goal_pose", PoseStamped, queue_size=10)
        rospy.Subscriber("/uav/trajectory", PathMsg, self._on_traj, queue_size=1)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/rate), self._tick)

    def _on_pose(self, msg):
        q = msg.pose.orientation
        try:
            yaw = quaternion_yaw((q.x,q.y,q.z,q.w))
        except ValueError:
            return
        with self._lock:
            if self._yaw is None:
                self._yaw = YawPolicy(yaw, self._yaw_rate)

    @staticmethod
    def _snapshot(path, sequence, generation, epoch):
        start = path.header.stamp.to_sec()
        points = []
        for p in path.poses:
            q = p.pose.orientation
            points.append(Sample((p.pose.position.x,p.pose.position.y,p.pose.position.z),
                (0.,0.,0.),quaternion_yaw((q.x,q.y,q.z,q.w)),
                (p.header.stamp-path.header.stamp).to_sec()))
            if p.header.frame_id not in ("",path.header.frame_id):
                raise ValueError("mixed trajectory frames")
        return TimedTrajectory("path-compat",generation,epoch,sequence,start,tuple(points),path.header.frame_id)

    def _on_traj(self, msg):
        with self._lock:
            if not msg.poses:
                self._generation += 1
                self._buffer.begin("path-compat",self._generation,self._epoch)
                self._traj = None
                return
            self._sequence += 1
            try:
                trajectory = self._snapshot(msg,self._sequence,self._generation,self._epoch)
                if self._buffer.context is None:
                    # A ROS clock reset requires a fresh message and playback epoch.
                    self._epoch = uuid.uuid4().hex
                    self._buffer.begin("path-compat",self._generation,self._epoch)
                    trajectory = self._snapshot(msg,self._sequence,self._generation,self._epoch)
                if not self._buffer.accept(trajectory,rospy.Time.now().to_sec()):
                    rospy.logwarn("[follower] Rejected path: %s",self._buffer.reason)
                    return
            except (ValueError,TypeError) as exc:
                rospy.logerr("[follower] Invalid timed path: %s",exc)
                return
            self._traj = copy.deepcopy(msg)

    def _tick(self, _event):
        with self._lock:
            now = rospy.Time.now().to_sec()
            sample = self._buffer.sample(now)
            if sample is None or self._yaw is None:
                return
            dt = 0.0 if self._yaw_last_time is None else max(0.,now-self._yaw_last_time)
            self._yaw_last_time = now
            yaw = self._yaw.update(sample.yaw,dt)
            pose = PoseStamped()
            pose.header = Header(stamp=rospy.Time.from_sec(now),frame_id="map")
            pose.pose.position = Point(*sample.position)
            pose.pose.orientation = Quaternion(*quaternion(yaw))
            self._goal_pub.publish(pose)

    @staticmethod
    def _interpolate(path, elapsed):
        snapshot = TrajectoryFollower._snapshot(path,0,0,"interpolation")
        sample = snapshot.sample(snapshot.start+elapsed)
        pose = PoseStamped()
        pose.pose.position = Point(*sample.position)
        pose.pose.orientation = Quaternion(*quaternion(sample.yaw))
        return pose

    @staticmethod
    def _traj_duration(path):
        return (path.poses[-1].header.stamp-path.header.stamp).to_sec() if path.poses else 0.

if __name__ == "__main__":
    TrajectoryFollower()
    rospy.spin()
