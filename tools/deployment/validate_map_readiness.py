#!/usr/bin/env python3
"""Exercise completed EGO fusion on an isolated master, without flight outputs."""
import argparse,json,math,threading,time
from pathlib import Path
import numpy as np
import rospy
from ego_planner.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    if rospy.get_param('/use_sim_time',False) is not True:
        raise RuntimeError('isolated controllable clock required')
    rospy.init_node('map_readiness_validation',anonymous=True)
    pubs={n:rospy.Publisher(t,c,queue_size=3) for n,t,c in [
        ('clock','/clock',Clock),('odom','/mavros/local_position/odom',Odometry),
        ('pose','/uav/planning/camera_pose',PoseStamped),('image','/uav/planning/depth',Image)]}
    control={'mode':'valid','z':-.18}; events=[];stop=threading.Event();checks=[]
    rospy.Subscriber('/uav/ego/status',GoalStatus,
        lambda m:events.append((time.monotonic(),m.map_ready,m.status)),queue_size=30)
    pixels=np.full((480,640),3.,dtype=np.float32)
    raw={k:a.tobytes() for k,a in [('valid',pixels),('zero',np.zeros_like(pixels)),
                                  ('nan_image',np.full_like(pixels,np.nan))]}
    counts={'images':0,'clock_ticks':0}
    def feed():
        sim=100.
        while not stop.is_set() and not rospy.is_shutdown():
            mode=control['mode']
            if mode!='freeze':sim+=.0666666667;counts['clock_ticks']+=1
            stamp=rospy.Time.from_sec(sim)
            pubs['clock'].publish(Clock(clock=stamp))
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='map'
            odom.child_frame_id='base_link';odom.pose.pose.position.z=.8
            odom.pose.pose.orientation.w=1.;pubs['odom'].publish(odom)
            if mode!='silent':
                p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='map'
                p.pose.position.z=control['z']
                if mode=='nan_position':p.pose.position.x=float('nan')
                q=p.pose.orientation;q.x=-.5;q.y=.5;q.z=-.5;q.w=.5
                if mode=='bad_quaternion':q.x=q.y=q.z=q.w=0.
                if mode=='wrong_frame':p.header.frame_id='world'
                image=Image();image.header.stamp=stamp;image.header.frame_id='camera_link_optical'
                image.height=480;image.width=640;image.encoding='32FC1';image.step=2560
                image.data=raw.get(mode,raw['valid'])
                if mode=='stale':
                    p.header.stamp=rospy.Time.from_sec(sim-.8);image.header.stamp=p.header.stamp
                pubs['pose'].publish(p);pubs['image'].publish(image);counts['images']+=1
            time.sleep(.0666666667)
    worker=threading.Thread(target=feed,daemon=True);worker.start()
    def expect(name,ready,mode='valid',z=-.18,seconds=4.):
        control.update(mode=mode,z=z);start=time.monotonic();before=dict(counts)
        while time.monotonic()-start<seconds:
            recent=[v for v in events if v[0]>=start+.15]
            if len(recent)>=4 and all(e[1] is ready for e in recent[-4:]):
                details=dict(name=name,passed=True,ready=ready,
                    elapsed_s=round(time.monotonic()-start,3),images=counts['images']-before['images'])
                if mode=='freeze':
                    assert details['images']>=5,'image delivery must continue while fusion timer is paused'
                checks.append(details);print(json.dumps(details),flush=True);return
            time.sleep(.04)
        raise AssertionError((name,ready,events[-8:]))
    error=None
    try:
        expect('negative_ground_height_can_fuse',True,seconds=12.)
        for mode,z in [('valid',-.65),('valid',3.2),('zero',-.18),('nan_image',-.18),
                       ('bad_quaternion',-.18),('nan_position',-.18),
                       ('wrong_frame',-.18),('stale',-.18),('silent',-.18),('freeze',-.18)]:
            label='below_map' if z<-.5 else 'above_map' if z>3. else mode
            expect('reject_'+label,False,mode,z)
            expect('recover_'+label,True)
        expect('working_height_can_fuse',True,z=1.6)
    except Exception as exc:error=repr(exc);print('FAIL',error,flush=True)
    finally:
        stop.set();worker.join(timeout=3)
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(dict(passed=error is None,error=error,checks=checks),indent=2)+'\n')
    raise SystemExit(error is not None)
if __name__=='__main__':main()
