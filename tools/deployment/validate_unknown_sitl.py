#!/usr/bin/python3
"""Verify unknown-space protection and input recovery on the actual project SITL."""
import argparse,json,math,os,signal,time,xmlrpc.client
from pathlib import Path
from types import SimpleNamespace
import rosgraph,rospy
from ego_planner.msg import Bspline
from gazebo_msgs.srv import GetLinkState,SpawnModel
from geometry_msgs.msg import Point,Pose
from std_msgs.msg import String
from validate_planning_sitl import BODY,PlanningTrial,decode
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11311'
rospy.init_node('unknown_space_sitl_validation',anonymous=True)
assert rospy.get_param('/use_sim_time') is True
world=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState);assert world(BODY,'world').success
trial=PlanningTrial(SimpleNamespace(mock=False,live=False,yaw=0.,target='known-space fixture'))
ready=[];splines=[];paused=None;error=None
rospy.Subscriber('/uav/planning/readiness',String,lambda m:ready.append(json.loads(m.data)),queue_size=10)
rospy.Subscriber('/uav/planning/trajectory',Bspline,lambda m:splines.append(m),queue_size=10)
replay=rospy.Publisher('/uav/planning/trajectory',Bspline,queue_size=1)
try:
 trial.takeoff();p,_=decode(trial.odom.pose.pose)
 name=trial.new('outside_view',waypoint=(p[0]-.8,p[1],1.6))
 result=trial.wait_phase(name,('DONE','FAILED'),32.)
 phases=[d for _,k,d in trial.trace if k=='mission' and d['mission_id']==name]
 assert result['phase']=='FAILED' and result['reason']=='unknown_trajectory',result
 assert not any(d['phase']=='TRACK' for d in phases)
 assert sum(d['phase']=='OBSERVE_MAP' for d in phases)==1
 assert trial.flight['mode']=='HOLD' and trial.flight['context'] is None
 trial.check('unknown_never_tracks_and_exits_after_one_observation')
 p,_=decode(trial.odom.pose.pose);trial.move('known_free',(p[0]+.65,p[1],1.6),35.)
 assert splines;old=splines[-1]
 real=world(BODY,'world').link_state.pose.position
 pose=Pose(position=Point(real.x+1.2,real.y,1.3));pose.orientation.w=1.
 geometry='<geometry><box><size>0.35 0.6 2.6</size></box></geometry>'
 sdf='<sdf version="1.6"><model name="guard_box"><static>true</static><link name="body"><collision name="c">'+geometry+'</collision><visual name="v">'+geometry+'</visual></link></model></sdf>'
 assert rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)('guard_box',sdf,'',pose,'world').success
 time.sleep(1.8);p,_=decode(trial.odom.pose.pose)
 name=trial.new('occupied',waypoint=(p[0]+1.2,p[1],1.6))
 result=trial.wait_phase(name,('DONE','FAILED'),25.)
 assert result['phase']=='FAILED' and result['segment_count']==0,result
 replies=[d for _,k,d in trial.trace if k=='plan_result' and d['mission_id']==name]
 assert any(d['reason'] in ('invalid_or_occupied_goal','occupied_trajectory') for d in replies),replies
 trial.check('occupied_goal_rejected',reason=replies[-1]['reason'])
 uri=rosgraph.Master(rospy.get_name()).lookupNode('/camera_pose_bridge')
 code,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name());assert code==1
 assert b'camera_pose_bridge.py' in Path('/proc/%d/cmdline'%pid).read_bytes()
 os.kill(pid,signal.SIGSTOP);paused=pid
 trial.wait(lambda:ready and not ready[-1]['ready'],5.,'stale map rejected')
 p,_=decode(trial.odom.pose.pose);name=trial.new('stale_map',waypoint=(p[0]+.4,p[1],1.6))
 result=trial.wait_phase(name,('DONE','FAILED'),22.)
 assert result['phase']=='FAILED' and result['segment_count']==0,result
 assert not any(k=='plan_request' and d['mission_id']==name for _,k,d in trial.trace)
 trial.check('stale_map_cannot_authorize_translation',reason=result['reason'])
 os.kill(paused,signal.SIGCONT);paused=None
 trial.wait(lambda:ready and ready[-1]['ready'],10.,'fresh fusion recovers')
 start=time.monotonic();replay.publish(old);time.sleep(2.)
 feedback=[d for t,k,d in trial.trace if t>=start and k=='flight']
 assert feedback and all(d['mode']=='HOLD' and d['context'] is None for d in feedback),feedback[-3:]
 assert trial.mission['mission_id']==name and trial.mission['phase']=='FAILED'
 trial.check('restored_input_does_not_revive_old_mission_or_replayed_spline')
 p,_=decode(trial.odom.pose.pose);trial.move('fresh_after_recovery',(p[0]+.4,p[1],1.6),35.)
 trial.check('new_explicit_mission_can_execute_after_fresh_fusion')
except Exception as exc:error=repr(exc);print('FAIL',error,flush=True)
finally:
 if paused:os.kill(paused,signal.SIGCONT)
 try:
  trial.stop_and_land()
  assert not trial.state.armed and trial.landed.landed_state==1
  trial.check('landed_disarmed')
 except Exception as exc:error=(error or '')+' CLEANUP '+repr(exc)
 (a.out/'results.json').write_text(json.dumps(dict(passed=error is None,error=error,checks=trial.checks,final_armed=trial.state.armed),indent=2)+'\n')
 (a.out/'trace.json').write_text(json.dumps(trial.trace)+'\n')
raise SystemExit(error is not None)
