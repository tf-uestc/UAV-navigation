#!/usr/bin/env python3
"""Exercise the production executor spline callback with isolated trajectories and odometry."""
import argparse
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import threading

import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/uav_vln_bringup/scripts'))
from flight_executor import FlightExecutor
from navigation_core.control import ControlArbiter, Reference


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    # No ROS master and no service connections: run the real message callback
    # with the actual ROS message types and an in-memory acknowledgement sink.
    rospy.rostime.set_rostime_initialized(True)
    rospy.rostime._set_rostime(rospy.Time.from_sec(100.))
    results=[]
    for label,mismatch in [('accepted_first',False),('discontinuous_replan',True)]:
        node=object.__new__(FlightExecutor)
        node.lock=threading.RLock()
        node.session='isolated-executor'
        node.bridge=SimpleNamespace(anchor=Reference((0.,0.,1.6),0.))
        node.control=ControlArbiter()
        node.control.begin('test',1,'epoch',node.bridge.anchor)
        node.gateway='test-gateway'
        node.planner=(node.gateway,'epoch')
        node.planner_ready=lambda _:True
        node.fence=None
        node.handover_accel=.35
        node.plan_deadline=None
        node.owned=True
        odom=Odometry()
        odom.header.stamp=rospy.Time.from_sec(100.)
        q=odom.pose.pose.orientation
        q.z,q.w=math.sin(math.pi/4),math.cos(math.pi/4)
        odom.twist.twist.linear.x=.25
        node.odom=odom,0.
        node.feedback=lambda wall,stamp:SimpleNamespace(position=(0.,0.,1.6),armed=True,mode='OFFBOARD')
        stops=[]
        def hold(feedback,reason):
            stops.append(reason)
            node.control.cancel(node.bridge.anchor)
        node.hold=hold
        acks=[]
        node.trajectory_ack=SimpleNamespace(publish=lambda m:acks.append(json.loads(m.data)))
        def submit(tid,vy):
            msg=Bspline(mission_id='test',goal_id=1,planner_session_id='epoch',traj_id=tid,
                        start_time=odom.header.stamp,order=3,knots=[0.]*4+[3.]*4,
                        pos_pts=[Point(0,y,1.6) for y in (0.,vy,.50,.50)])
            node.spline(msg)
        submit(1,.25)
        assert node.control.buffer.active and acks[-1]['accepted'] and acks[-1]['traj_id']==1,acks
        if mismatch:
            submit(2,-.25)
            assert not acks[-1]['accepted'] and acks[-1]['reason']=='velocity_discontinuity',acks
            assert stops==['discontinuous_trajectory'] and node.control.buffer.active is None
        else:
            submit(1,.25)
            assert not acks[-1]['accepted'] and acks[-1]['reason']=='old_trajectory'
            assert node.control.buffer.active.traj_id==1 and not stops
        results.append(dict(case=label,passed=True,dispositions=acks,stops=stops))
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(dict(passed=True,checks=results),indent=2)+'\n')
    print('PASS rotation-aware measured velocity, current ID, old ID and discontinuous replacement')


if __name__=='__main__':
    main()
