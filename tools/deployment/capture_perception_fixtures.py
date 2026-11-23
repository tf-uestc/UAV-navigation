#!/usr/bin/env python3
"""Collect real RGB-D foreground/absent fixtures while holding in project SITL."""
import argparse,collections,json,math,os,time
from pathlib import Path
from types import SimpleNamespace
import cv2,numpy as np,rospy,message_filters,tf2_ros
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState,SetModelState
from sensor_msgs.msg import Image,CameraInfo
from tf.transformations import quaternion_matrix
from validate_planning_sitl import PlanningTrial,decode

def matrix(p,q):
 m=quaternion_matrix([q.x,q.y,q.z,q.w]);m[:3,3]=[p.x,p.y,p.z];return m
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
 if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11311':raise RuntimeError('SITL master required')
 rospy.init_node('perception_fixture_capture',anonymous=True);args.out.mkdir(parents=True,exist_ok=True)
 trial=PlanningTrial(SimpleNamespace(mock=False,live=False,yaw=None,target='fixture capture'))
 bridge=CvBridge();buffer=tf2_ros.Buffer(rospy.Duration(30));listener=tf2_ros.TransformListener(buffer)
 queue=collections.deque(maxlen=3)
 subs=[message_filters.Subscriber(t,c,queue_size=2) for t,c in [
   ('/iris_depth_camera/camera/rgb/image_raw',Image),('/iris_depth_camera/camera/depth/image_raw',Image),
   ('/iris_depth_camera/camera/depth/camera_info',CameraInfo)]]
 sync=message_filters.TimeSynchronizer(subs,20);sync.registerCallback(lambda *ms:queue.append(ms))
 get=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
 move=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
 original=get('red_tripod_A','world');assert original.success
 rows=[];error=None
 try:
  trial.takeoff();time.sleep(2.)
  for distance,kind in [(3.6,'positive'),(2.4,'positive'),(1.6,'positive'),(-3.6,'negative')]:
   body=trial.world('iris_depth_camera::iris::base_link','world');assert body.success
   pose=body.link_state.pose;_,yaw=decode(pose)
   target=get('red_tripod_A','world');assert target.success
   target.pose.position.x=pose.position.x+distance*math.cos(yaw)
   target.pose.position.y=pose.position.y+distance*math.sin(yaw)
   assert move(ModelState(model_name='red_tripod_A',pose=target.pose,reference_frame='world')).success
   time.sleep(.7);queue.clear()
   for repeat in range(2):
    end=time.monotonic()+8.
    while time.monotonic()<end:
     time.sleep(.04)
     if not queue:continue
     rgb,depth,info=queue.pop();queue.clear()
     _,_=decode(trial.odom.pose.pose)
     v=trial.odom.twist.twist.linear
     if math.hypot(v.x,v.y,v.z)>.15 or abs((rospy.Time.now()-rgb.header.stamp).to_sec())>.10:continue
     color=bridge.imgmsg_to_cv2(rgb,'rgb8');r=color.astype(np.float32)
     red=(r[:,:,0]>110)&(r[:,:,0]>r[:,:,1]*1.5)&(r[:,:,0]>r[:,:,2]*1.5)
     pixels=int(red.sum())
     if kind=='positive' and pixels<20 or kind=='negative' and pixels>2:continue
     tf=buffer.lookup_transform('base_link','camera_link_optical',rgb.header.stamp,rospy.Duration(.1)).transform
     before=rospy.Time.now().to_sec();truth=trial.world('iris_depth_camera::iris::base_link','world')
     after=rospy.Time.now().to_sec()
     if not truth.success or after-before>.04:continue
     camera=matrix(truth.link_state.pose.position,truth.link_state.pose.orientation)@matrix(tf.translation,tf.rotation)
     ident=kind+'-'+str(abs(distance)).replace('.','p')+'-'+str(repeat+1)
     folder=args.out/ident;folder.mkdir()
     cv2.imwrite(str(folder/'rgb.png'),cv2.cvtColor(color,cv2.COLOR_RGB2BGR))
     raw=bridge.imgmsg_to_cv2(depth,'passthrough').astype(np.float32)
     if depth.encoding=='16UC1':raw*=.001
     np.save(folder/'depth.npy',raw)
     meta=dict(label=kind,nominal_distance_m=abs(distance),red_pixels=pixels,K=list(info.K),
       width=info.width,height=info.height,capture_stamp=rgb.header.stamp.to_sec(),
       world_from_optical=camera.tolist(),
       target_root_position=[target.pose.position.x,target.pose.position.y,target.pose.position.z],
       target_cylinder_radius=.05,target_cylinder_height=.9,source='real static RGB-D during project SITL hover')
     (folder/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
     rows.append(dict(fixture=ident,label=kind,red_pixels=pixels))
     print('CAPTURE',json.dumps(rows[-1]),flush=True);time.sleep(.25);break
    else:raise RuntimeError('no valid synchronized '+kind+' fixture at '+str(distance))
 except Exception as exc:error=repr(exc);print('FAIL',error,flush=True)
 finally:
  try:trial.stop_and_land()
  except Exception as exc:error=(error or '')+' cleanup '+repr(exc)
  try:move(ModelState(model_name='red_tripod_A',pose=original.pose,reference_frame='world'))
  except rospy.ServiceException:pass
  (args.out/'results.json').write_text(json.dumps(dict(passed=error is None,rows=rows,error=error,
    final_armed=trial.state.armed if trial.state else None),indent=2)+'\n')
 raise SystemExit(error is not None)
if __name__=='__main__':main()
