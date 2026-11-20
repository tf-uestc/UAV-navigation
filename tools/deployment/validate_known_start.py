#!/usr/bin/env python3
"""Test start-body UNKNOWN handling on real EGO without a flight controller."""
import argparse,json,math,threading,time
from pathlib import Path
import numpy as np,rospy
from ego_planner.msg import Bspline,GoalCommand,GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler,quaternion_multiply

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    rospy.init_node('known_start_validation',anonymous=True)
    op=rospy.Publisher('/mavros/local_position/odom',Odometry,queue_size=2)
    pp=rospy.Publisher('/uav/planning/camera_pose',PoseStamped,queue_size=2)
    ip=rospy.Publisher('/uav/planning/depth',Image,queue_size=2)
    cp=rospy.Publisher('/uav/ego/command',GoalCommand,queue_size=2)
    events=[];splines=[];stop=threading.Event();state={'depth':3.};checks=[]
    rospy.Subscriber('/uav/ego/status',GoalStatus,lambda m:events.append(m),queue_size=30)
    rospy.Subscriber('/planning/bspline',Bspline,lambda m:splines.append(m),queue_size=20)
    bq=quaternion_from_euler(0,0,math.pi/2)
    cq=quaternion_multiply(bq,[-.5,.5,-.5,.5])
    def feed():
        while not stop.is_set() and not rospy.is_shutdown():
            stamp=rospy.Time.now();od=Odometry();od.header.frame_id='map';od.header.stamp=stamp
            od.child_frame_id='base_link';od.pose.pose.position.z=1.6
            q=od.pose.pose.orientation;q.x,q.y,q.z,q.w=bq
            pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='map'
            pose.pose.position.y=.1;pose.pose.position.z=1.6
            q=pose.pose.orientation;q.x,q.y,q.z,q.w=cq
            image=Image();image.header.stamp=stamp;image.header.frame_id='camera_link_optical'
            image.width=640;image.height=480;image.encoding='32FC1';image.step=2560
            image.data=np.full((480,640),state['depth'],np.float32).tobytes()
            op.publish(od);pp.publish(pose);ip.publish(image);time.sleep(.08)
    thread=threading.Thread(target=feed,daemon=True);thread.start()
    def wait(fn,seconds,label):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            v=fn()
            if v:return v
            time.sleep(.025)
        raise AssertionError(label)
    def command(name,y,cmd='GOAL'):
        m=GoalCommand();m.header.stamp=rospy.Time.now();m.header.frame_id='map'
        m.planner_session_id=events[-1].planner_session_id;m.mission_id=name;m.goal_id=1
        m.command=cmd;m.goal.y=y;m.goal.z=1.6;cp.publish(m)
    def result(name,allowed):
        return next((m for m in reversed(events) if m.mission_id==name and m.status in allowed),None)
    def check(name,**kwargs):
        row=dict(name=name,passed=True,**kwargs);checks.append(row);print(json.dumps(row),flush=True)
    error=None
    try:
        wait(lambda:events and events[-1].map_ready and cp.get_num_connections(),12,'map readiness')
        command('known_start_forward',.8)
        wait(lambda:any(s.mission_id=='known_start_forward' and not s.emergency_stop for s in splines),8,'forward trajectory across current body blind region')
        check('forward_path_out_of_current_body_into_observed_space')
        command('known_start_forward',.8,'CANCEL')
        wait(lambda:result('known_start_forward',('CANCELLED',)),3,'cancel forward fixture')
        command('outside_body_unknown',-.8)
        m=wait(lambda:result('outside_body_unknown',('FAILED','REJECTED')),8,'unobserved reverse path rejection')
        assert m.reason=='unknown_trajectory',m
        assert not any(s.mission_id=='outside_body_unknown' for s in splines)
        check('unknown_outside_start_body_still_rejected',reason=m.reason)
        state['depth']=.3;time.sleep(2.)
        command('occupied_inside_body',.2)
        m=wait(lambda:result('occupied_inside_body',('FAILED','REJECTED')),8,'occupied goal inside prefix')
        assert m.reason=='invalid_or_occupied_goal',m
        assert not any(s.mission_id=='occupied_inside_body' for s in splines)
        check('inflated_occupancy_inside_body_is_never_exempted',reason=m.reason)
    except Exception as exc:error=repr(exc);print('FAIL',error,flush=True)
    finally:
        stop.set();thread.join(timeout=3)
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(dict(passed=error is None,error=error,checks=checks),indent=2)+'\n')
    raise SystemExit(error is not None)
if __name__=='__main__':main()
