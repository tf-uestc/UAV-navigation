#!/usr/bin/env python3
"""Reproduce the legacy HOLD transition and audit corrected ROS/SITL traces."""
import argparse
import bisect
import json
import math
import os
import runpy
import threading
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from std_msgs.msg import Header, String
from navigation_core.yaw import angle_delta, quaternion
from validate_flight_execution import decode


def wait(predicate,timeout):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        value=predicate()
        if value:
            return value
        time.sleep(.025)
    raise AssertionError('heading evidence missing')


def legacy(source):
    assert os.environ['ROS_MASTER_URI']=='http://127.0.0.1:11317'
    rospy.init_node('legacy_heading_validation',anonymous=True)
    module=runpy.run_path(str(source))
    rospy.Service('/mavros/cmd/arming',CommandBool,lambda _:CommandBoolResponse(success=False))
    rospy.Service('/mavros/set_mode',SetMode,lambda _:SetModeResponse(mode_sent=False))
    old=module['SetpointBridge']()
    state=rospy.Publisher('/mavros/state',State,queue_size=1,latch=True)
    pose=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=2)
    target=rospy.Publisher('/uav/goal_pose',PoseStamped,queue_size=2)
    cmd=rospy.Publisher('/uav/state_cmd',String,queue_size=1)
    sent=[]
    rospy.Subscriber('/mavros/setpoint_position/local',PoseStamped,
                     lambda msg:sent.append((time.monotonic(),decode(msg.pose)[1])),queue_size=10)
    wait(lambda: all(p.get_num_connections() for p in (state,pose,target,cmd)),6)
    state.publish(State(connected=True,armed=True,mode='OFFBOARD'))
    wait(lambda: old.state.connected,3)
    results=[]
    for heading in (0,90,180,-90):
        yaw=math.radians(heading)
        current=PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'))
        current.pose.position=Point(0.,0.,1.6)
        current.pose.orientation=Quaternion(*quaternion(yaw))
        requested=PoseStamped(header=current.header)
        requested.pose.position=Point(.1,0.,1.6)
        requested.pose.orientation=current.pose.orientation
        pose.publish(current)
        target.publish(requested)
        wait(lambda: old.goal_pose and old.goal_pose.header.stamp==requested.header.stamp
             and old.current_pose.header.stamp==current.header.stamp,3)
        old.bridge_state=module['BridgeState'].FLYING
        before=time.monotonic()
        if not results:
            threading.Thread(target=old.run,daemon=True).start()
        first=wait(lambda:next((y for t,y in sent if t>=before and abs(angle_delta(y,yaw))<1e-6),None)
                   if heading!=0 else any(t>=before and abs(y)<1e-6 for t,y in sent),3)
        switch=time.monotonic()
        cmd.publish(String(data='HOVER'))
        wait(lambda:old.bridge_state==module['BridgeState'].HOVER,3)
        wait(lambda:any(t>=switch and abs(y)<1e-6 for t,y in sent),3)
        t,after=next((t,y) for t,y in sent if t>=switch and abs(y)<1e-6)
        measured=decode(old.current_pose.pose)[1]
        original=decode(old.goal_pose.pose)[1]
        results.append(dict(initial_yaw_deg=heading,goal_yaw_deg=math.degrees(original),
            pose_yaw_deg=math.degrees(measured),legacy_hover_setpoint_deg=math.degrees(after),
            step_deg=abs(math.degrees(angle_delta(after,yaw))),
            first_setpoint_ms=(t-switch)*1000))
    assert results[2]['step_deg']==180.
    assert abs(math.degrees(angle_delta(math.radians(-179),math.radians(179)))-2.)<1e-9
    return dict(passed=True,first_changed_topic='/mavros/setpoint_position/local',
                reason='legacy HOVER calls _make_pose without yaw',cases=results,wrap_test_deg=2.)


def corrected(folder):
    result=json.loads((folder/'results.json').read_text())
    trace=json.loads((folder/'trace.json').read_text())
    assert result['passed'] and result['final_armed'] is False
    mission_id=next(c['mission_id'] for c in result['checks'] if c['name']=='segmented_approach_completed')
    changes=[(t,d['phase'],d['intended_yaw']) for t,k,d in trace if k=='mission' and d['mission_id']==mission_id]
    waiting=[]
    for (begin,phase,yaw),(end,_phase,_yaw) in zip(changes,changes[1:]):
        if phase!='WAIT_VLM':
            continue
        samples=[d for t,k,d in trace if begin<=t<=end and k=='setpoint']
        if not samples:
            raise AssertionError('no setpoints while waiting for detector')
        actual=[d for t,k,d in trace if begin<=t<=end and k=='odom']
        assert all(math.dist(d['position'],samples[0]['position'])<1e-6 for d in samples)
        drift=max(math.dist(d['position'],samples[0]['position']) for d in actual)
        assert drift<=.3,drift
        waiting.append(dict(seconds=end-begin,position_error_m=drift,
                            yaw_error_deg=max(abs(math.degrees(angle_delta(d['yaw'],yaw))) for d in actual)))
    assert len(waiting)>=3 and max(d['yaw_error_deg'] for d in waiting)<=10.
    first=next(t for t,phase,_ in changes if phase=='WAIT_VLM')
    end=next(t for t,phase,_ in changes if phase=='DONE')+9.
    sent=[(t,d) for t,k,d in trace if first<=t<=end and k=='setpoint']
    pose=[d for t,k,d in trace if first<=t<=end and k=='odom']
    target=[(t,d) for t,k,d in trace if first<=t<=end and k=='px4_target']
    assert len(sent)>30 and len(target)>20
    stamps={d['stamp']:d for _,d in sent}
    downstream=[abs(math.degrees(angle_delta(d['yaw'],stamps[d['stamp']]['yaw'])))
                for t,k,d in trace if first<=t<=end and k=='goal_pose' and d['stamp'] in stamps]
    assert len(downstream)>30 and max(downstream)<1e-6
    times=[t for t,_ in sent]
    differences=[]
    for t,d in target:
        i=min(bisect.bisect_left(times,t),len(sent)-1)
        nearest=min(sent[max(0,i-1):i+1],key=lambda sample:abs(sample[0]-t))
        if abs(nearest[0]-t)<=.15:
            differences.append(abs(math.degrees(angle_delta(d['yaw'],nearest[1]['yaw']))))
    peak_step=max(abs(math.degrees(angle_delta(b['yaw'],a['yaw']))) for (_,a),(_,b) in zip(sent,sent[1:]))
    max_actual=max(abs(math.degrees(angle_delta(d['yaw'],pose[0]['yaw']))) for d in pose)
    assert len(differences)>20 and max(differences)<10. and peak_step<10. and max_actual<10.
    return dict(passed=True,initial_yaw_deg=math.degrees(pose[0]['yaw']),
                max_setpoint_shortest_step_deg=peak_step,max_actual_heading_excursion_deg=max_actual,
                max_goal_pose_vs_setpoint_deg=max(downstream),max_px4_vs_setpoint_deg=max(differences),
                aligned_px4_samples=len(differences),detector_waits=waiting)


def main():
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--legacy',type=Path)
    group.add_argument('--corrected',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    data=legacy(args.legacy) if args.legacy else corrected(args.corrected)
    args.out.write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(data))
    if args.legacy:
        rospy.signal_shutdown('verified')


if __name__=='__main__':
    main()
