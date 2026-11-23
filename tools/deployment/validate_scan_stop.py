#!/usr/bin/env python3
"""Regression for active braking before the next scan; isolated ROS only."""
import argparse,importlib.util,json,math,os,sys,time
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11318':
        raise RuntimeError('requires isolated test master on 11318')
    scripts=Path(__file__).resolve().parents[2]/'ws/src/uav_vln_bringup/scripts'
    sys.path.insert(0,str(scripts))
    from segment_mission import SegmentMission
    from navigation_core.contracts import DetectionResult
    rospy.set_param('/uav/segment_config/fence',dict(x_min=-6.,x_max=6.,y_min=-4.,y_max=4.,z_min=1.1,z_max=2.35))
    node=SegmentMission();commands=[];checks=[]
    rospy.Subscriber('/uav/flight/command',String,lambda m:commands.append(json.loads(m.data)),queue_size=20)
    time.sleep(.25)
    def feed(seq,confirmed,speed):
        m=Odometry();m.header.stamp=rospy.Time.now();m.header.frame_id='map';m.child_frame_id='base_link'
        m.pose.pose.position.z=1.6
        q=m.pose.pose.orientation;q.x,q.y,q.z,q.w=quaternion_from_euler(0,0,math.pi/4)
        m.twist.twist.linear.x=speed;node.on_odom(m)
        node.on_flight(String(data=json.dumps(dict(flight_session_id='isolated-flight',seq=seq,mode='HOLD',
            owns_flight=True,ready=True,context=None,stop_id=seq,stop_confirmed=confirmed))))
    def absent(name,seq):
        feed(seq,False,.24)
        node.flight_session='isolated-flight';node.mission=name;node.target='red pole';node.mode='visual'
        node.scan_base=0.;node.scan_index=1;node.stop_id=seq;node.started=time.monotonic();node.last_ros=None
        node.registry.begin(name);key=node.registry.issue(node.target,time.monotonic(),42.)
        node.settled_at=rospy.Time.now().to_sec()-.2;node.phase='WAIT_VLM'
        result=DetectionResult(key,node.target,'FAILED',rospy.Time.now().to_sec(),rospy.Time.now().to_sec(),None,'map','target_not_found')
        node.on_detection(String(data=result.to_json()));time.sleep(.08)
    def check(name):
        checks.append(dict(name=name,passed=True));print('PASS',name,flush=True)
    error=None
    try:
        from flight_executor import FlightExecutor
        from navigation_core.bridge import BridgeDecisions,Feedback
        from navigation_core.control import ControlArbiter,Reference
        import threading
        executor=object.__new__(FlightExecutor)
        executor.lock=threading.RLock();executor.session='anchor-fixture';executor.sequence=1
        executor.control=ControlArbiter();executor.gateway=None;executor.owned=True
        executor.bridge=BridgeDecisions();executor.bridge.mode='HOLD'
        executor.bridge.anchor=Reference((0.,0.,1.6),0.)
        executor.stop_id=1
        current=[Feedback((.25,.1,1.6),.5,True,True,'OFFBOARD',False,0.,.2)]
        executor.feedback=lambda wall,stamp:current[0]
        def hover(seq):
            executor.command(String(data=json.dumps(dict(flight_session_id=executor.session,seq=seq,command='HOVER'))))
        hover(2)
        assert executor.bridge.anchor==Reference(current[0].position,current[0].yaw),'new HOVER retained old hold anchor'
        check('new_stop_request_captures_current_anchor_once')
        anchor=executor.bridge.anchor
        current[0]=Feedback((.4,.3,1.6),.7,True,True,'OFFBOARD',False,0.,.1)
        hover(2)
        assert executor.bridge.anchor==anchor
        executor.bridge.tick(current[0],time.monotonic())
        assert executor.bridge.anchor==anchor
        check('duplicate_stop_and_feedback_do_not_chase_the_measured_pose')
        hover(3)
        assert executor.bridge.anchor==Reference(current[0].position,current[0].yaw)
        check('later_explicit_stop_can_capture_a_new_anchor')
        absent('resume-scan',3)
        assert commands and commands[-1]['command']=='HOVER','no active HOVER after not-found while prior hold is unconfirmed'
        check('not_found_requests_fresh_brake_instead_of_waiting_on_old_hold')
        stop_id=node.stop_id
        feed(stop_id-1,True,0.);node.tick(time.monotonic());time.sleep(.03)
        assert not any(d['command']=='TURN' for d in commands)
        check('old_stop_confirmation_cannot_authorize_scan')
        feed(stop_id,False,.24);node.tick(time.monotonic());time.sleep(.03)
        assert node.phase=='SETTLE' and not any(d['command']=='TURN' for d in commands)
        check('moving_or_unconfirmed_aircraft_cannot_turn')
        feed(stop_id,True,0.);node.tick(time.monotonic());time.sleep(.20)
        feed(stop_id,True,0.);node.tick(time.monotonic());time.sleep(.05)
        turns=[d for d in commands if d['command']=='TURN']
        assert len(turns)==1 and abs(turns[0]['yaw']+math.pi/4)<1e-6,(node.phase,commands)
        check('fresh_confirmed_stop_resumes_the_next_scan_offset')
        previous_turns=len(turns);absent('cannot-stop',20);stop_id=node.stop_id
        feed(stop_id,False,.24);node.tick(time.monotonic())
        feed(stop_id,False,.24);node.tick(node.deadline+.01);time.sleep(.08)
        assert node.phase=='FAILED' and node.reason=='stop_confirmation_timeout'
        assert len([d for d in commands if d['command']=='TURN'])==previous_turns
        assert commands[-1]['command']=='LAND',commands
        check('persistent_motion_times_out_and_requests_landing')
        original_feedback=node.feedback
        node.feedback=lambda:((0.,0.,1.6),0.,0.)
        node.detour=[];node.final_goal=(.11,0.,1.6);node.phase='SETTLE'
        captures=[];node.brake=lambda after,*a:captures.append(after)
        node.plan_segment('approach')
        assert captures==['DETECT'] and node.phase!='DONE','arrival deadband declared visual success without final observation'
        check('visual_arrival_deadband_requests_observation_instead_of_unverified_done')
        node.feedback=original_feedback;feed(31,True,0.)
        node.mission='off-axis-target';node.mode='visual';node.target='red pole'
        node.estimator.reset();node.acquisition_point=(4.,0.,.3)
        node.registry.begin(node.mission);key=node.registry.issue(node.target,time.monotonic(),42.)
        node.settled_at=rospy.Time.now().to_sec()-.2;node.phase='WAIT_VLM'
        decisions=[];node.brake=lambda after,*a:decisions.append(after)
        node.plan_segment=lambda *a,**k:decisions.append('PLAN')
        result=DetectionResult(key,node.target,'SUCCEEDED',rospy.Time.now().to_sec(),
            rospy.Time.now().to_sec(),(4.,0.,.3),'map','')
        node.on_detection(String(data=result.to_json()))
        assert decisions==['ALIGN'],'translation planned before aligning off-axis target: '+str(decisions)
        check('off_axis_target_brakes_for_alignment_before_translation')
        node.target_heading=0.;node.detection_retried=True
        node.command_flight=lambda *a,**k:99
        node.budget.begin(time.monotonic());node.budget.calls=5
        budget_state=(node.budget.calls,node.budget.started)
        node.align_target()
        assert (node.budget.calls,node.budget.started)==budget_state
        assert node.phase=='ALIGN_TARGET' and node.detection_retried is False,'new view reused old view retry flag'
        check('new_aligned_view_has_one_fresh_format_retry_without_resetting_call_budget')
        node.phase='TRACK';node.kind='manual';node.detour_used=False;node.recovery_count=0
        node.epoch=('gateway','planner');node.goal_id=1;node.mission='dynamic-fixture'
        actions=[]
        def brake(after,reason=''):
            actions.append((after,reason));node.phase='BRAKE'
        node.brake=brake
        event=dict(mission_id=node.mission,goal_id=1,gateway_session_id='gateway',
            planner_session_id='planner',status='FAILED',reason='occupied_trajectory')
        node.on_plan(String(data=json.dumps(event)))
        assert actions==[('REPLAN','planner_stop')] and node.recovery_count==1,actions
        check('active_occupied_trajectory_requires_braking_before_bounded_detour')
        actions.clear();feed(32,True,0.);node.phase='TRACK';node.recovery_count=0
        node.started=time.monotonic();node.last_ros=None
        status=node.flight_status().copy();status['reason']='planner_occupied_trajectory'
        node.flight_status=lambda:status
        node.tick(time.monotonic())
        assert actions==[('REPLAN','planner_stop')] and node.recovery_count==1,actions
        check('flight_feedback_winning_callback_race_still_requests_safe_detour')
        node.plan_segment=SegmentMission.plan_segment.__get__(node)
        feed(33,True,0.);node.mode='manual';node.phase='SETTLE'
        node.detour=[(0.,1.,1.6),(1.,1.,1.6),(.8,0.,1.6)]
        node.detour_used=True;node.final_goal=(.8,0.,1.6);node.segment_count=1
        actions.clear();node.plan_segment('manual')
        assert actions==[('ALIGN_PLAN','')],'sideways staging goal authorized before camera observed it: '+str(actions)
        check('detour_staging_goal_requires_observation_heading_before_authorization')
        node.align_target(map_only=True)
        assert node.phase=='ALIGN_MAP' and node.heading_mode=='map_alignment'
        status.update(stop_id=99,stop_confirmed=True,context=None,mode='HOLD',owns_flight=True)
        node.feedback=lambda:((0.,0.,1.6),(0.,0.,0.),node.target_heading,0.,
            rospy.Time.now().to_sec(),time.monotonic())
        actions.clear()
        node.tick(time.monotonic());time.sleep(.25);node.tick(time.monotonic())
        assert node.phase=='BRAKE' and actions==[('OBSERVE_MAP','detour_view')],(node.phase,actions)
        check('map_only_turn_reanchors_hold_before_fusion_wait_without_visual_api')
        node.phase='WAIT_VLM';node.mode='visual';node.mission='verified-close-band'
        node.target='red pole';node.detour=[];node.estimator.reset()
        node.acquisition_point=(0.,2.51,.3);node.settled_at=rospy.Time.now().to_sec()-.1
        node.registry.begin(node.mission);key=node.registry.issue(node.target,time.monotonic(),42.)
        result=DetectionResult(key,node.target,'SUCCEEDED',rospy.Time.now().to_sec(),
            rospy.Time.now().to_sec(),(0.,2.51,.3),'map','')
        node.on_detection(String(data=result.to_json()))
        assert node.phase=='DONE' and node.reason=='observation_standoff_reached',node.phase
        check('fresh_observation_inside_arrival_band_finishes_without_micro_replanning')
    except Exception as exc:error=repr(exc);print('FAIL',error,flush=True)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(dict(passed=error is None,error=error,checks=checks,commands=commands),indent=2)+'\n')
    raise SystemExit(error is not None)
if __name__=='__main__':main()
