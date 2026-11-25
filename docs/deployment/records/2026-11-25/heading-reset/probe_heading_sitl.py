#!/usr/bin/python3
"""Cloud-free SITL probe of the actual takeoff, scan, RGB-D visibility and landing."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetLinkState,GetModelState,SetModelState
import message_filters
import numpy as np
import rosgraph
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from navigation_core.contracts import DetectionResult,RequestKey
from validate_search_sitl import SearchTrial,TARGET,decode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--expect-ready',action='store_true')
    args=p.parse_args()
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11311':
        p.error('requires isolated local Gazebo SITL')
    args.out.mkdir(parents=True,exist_ok=False)
    args.live,args.mock,args.yaw,args.target=True,False,0.,TARGET
    rospy.init_node('u03_cloud_free_probe',anonymous=True)
    trial=SearchTrial(args)
    todo=deque()
    frames=deque(maxlen=20)
    headings=deque(maxlen=2500)
    echo=rospy.Subscriber('/uav/instruction',String,lambda m:todo.append(json.loads(m.data)),queue_size=4)
    heading=rospy.Subscriber('/uav/sitl/heading_ready',String,lambda m:headings.append(json.loads(m.data)),queue_size=20)
    topics=[message_filters.Subscriber('/iris_depth_camera/camera/'+kind+'/image_raw',Image,queue_size=2)
            for kind in ('rgb','depth')]
    sync=message_filters.TimeSynchronizer(topics,5)
    sync.registerCallback(lambda a,b:frames.append((a,b)))
    bridge=CvBridge()
    reports=[]
    error=None
    try:
        trial.initial()
        get=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        place=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        marker=get('red_tripod_A','world')
        body=link('iris_depth_camera::iris::base_link','world')
        assert marker.success and body.success
        pos,yaw=decode(body.link_state.pose)
        angle=yaw-math.radians(54)
        marker.pose.position.x=pos[0]+3.6*math.cos(angle)
        marker.pose.position.y=pos[1]+3.6*math.sin(angle)
        assert place(ModelState(model_name='red_tripod_A',pose=marker.pose,reference_frame='world')).success
        owners=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        assert set(owners.get('/uav/detection_result',[]))=={rospy.get_name()},'real detector or duplicate fixture active'
        mission=trial.new('heading_gate_probe',takeoff=True)
        deadline=time.monotonic()+85.
        while len(reports)<3 and time.monotonic()<deadline and not rospy.is_shutdown():
            if trial.mission.get('mission_id')==mission and trial.mission.get('phase')=='FAILED':
                raise RuntimeError(trial.mission['reason'])
            if not todo:
                time.sleep(.025)
                continue
            request=todo[0]
            if request['mission_id']!=mission:
                todo.popleft()
                continue
            pair=next((pair for pair in list(frames) if pair[0].header.stamp.to_sec()>=request['capture_after']),None)
            if not pair:
                time.sleep(.02)
                continue
            todo.popleft()
            rgb,depth=pair
            stamp=rgb.header.stamp.to_sec()
            image=bridge.imgmsg_to_cv2(rgb,'bgr8')
            blue,green,red=np.moveaxis(image.astype(float),-1,0)
            depth_m=bridge.imgmsg_to_cv2(depth,'passthrough')
            if depth.encoding=='16UC1':
                depth_m=depth_m.astype(float)*.001
            good=(red>110)&(red>green*1.5)&(red>blue*1.5)&np.isfinite(depth_m)&(depth_m>1)&(depth_m<8)
            ys,xs=np.nonzero(cv2.erode(good.astype('uint8'),np.ones((3,3),'uint8')))
            cv2.imwrite(str(args.out/('request-%s.png'%request['request_id'])),image)
            offset=trial.mission['scan_offset_deg']
            live=link('iris_depth_camera::iris::base_link','world')
            assert live.success
            world_pose,world_yaw=decode(live.link_state.pose)
            prior=[h for h in list(headings) if 0<=stamp-h['stamp']<=.65]
            px4=prior[-1] if prior else None
            reports.append(dict(request_id=request['request_id'],scan=offset,stamp=stamp,
                red_interior_pixels=len(xs),red_width=(int(xs.max()-xs.min()+1) if len(xs) else 0),
                red_height=(int(ys.max()-ys.min()+1) if len(ys) else 0),
                red_depth_median_m=(float(np.median(depth_m[ys,xs])) if len(xs) else None),
                camera_frame=rgb.header.frame_id,px4=px4,world_body_z_m=world_pose[2],
                world_yaw_deg=math.degrees(world_yaw),odom_yaw_deg=math.degrees(decode(trial.odom.pose.pose)[1])))
            # All negative responses are predetermined. Neither poses nor image measurements control a maneuver.
            reason='target_not_found' if len(reports)<3 else 'heading_probe_complete'
            reply=DetectionResult(RequestKey(mission,request['request_id']),TARGET,'FAILED',stamp,
                                  rospy.Time.now().to_sec(),None,'map',reason)
            trial.detection.publish(String(data=reply.to_json()))
        assert [r['scan'] for r in reports]==[0,45,-45],reports
        assert trial.wait_phase(mission,('FAILED',),4)['reason']=='heading_probe_complete'
        assert not trial.mission_events(mission,'plan_request'),'unexpected translation'
        assert reports[-1]['red_interior_pixels']>=15 and reports[-1]['red_height']>reports[-1]['red_width'],reports
        assert 2.7<=reports[-1]['red_depth_median_m']<=4.2,reports
        if args.expect_ready:
            assert all(r['px4'] and r['px4']['ready'] for r in reports),reports
            assert len({r['px4']['counter'] for r in reports})==1,reports
    except Exception as exc:
        error=repr(exc)
    finally:
        try:
            trial.stop_and_land()
            assert trial.state and not trial.state.armed and trial.landed.landed_state==1
        except Exception as exc:
            error=(error or '')+' cleanup '+repr(exc)
        for sub in (echo,heading):
            sub.unregister()
        (args.out/'trace.json').write_text(json.dumps(trial.trace))
        (args.out/'result.json').write_text(json.dumps(dict(passed=error is None,error=error,
            requests=reports,headings=list(headings),final_armed=trial.state.armed if trial.state else None),indent=2))
    print(json.dumps(dict(passed=error is None,error=error,requests=reports)),flush=True)
    raise SystemExit(error is not None)


if __name__=='__main__':
    main()
