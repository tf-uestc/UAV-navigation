#!/usr/bin/python3
"""Read-only capture of exact RGB frames chosen for individual observations."""
import argparse,json,signal,threading,time
from collections import deque
from pathlib import Path
import cv2,rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
ap=argparse.ArgumentParser();ap.add_argument('--out',required=True,type=Path);args=ap.parse_args()
args.out.mkdir(parents=True,exist_ok=True)
rospy.init_node('observation_image_recorder',anonymous=True,disable_signals=True)
stop=threading.Event();lock=threading.Lock();active={};rows=[];recent=deque(maxlen=18)
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *a:stop.set())
bridge=CvBridge()
def request(m):
 d=json.loads(m.data)
 with lock:
  saved=dict(request=d,frames=[]);active[(d['mission_id'],d['request_id'])]=saved
  for frame in list(recent):
   stamp=frame.header.stamp.to_sec()
   if d['capture_after']<=stamp:
    name='%s-%s-%.3f.png'%(d['mission_id'],d['request_id'],stamp)
    cv2.imwrite(str(args.out/name),bridge.imgmsg_to_cv2(frame,'bgr8'))
    saved['frames'].append(dict(stamp=stamp,file=name))
def image(m):
 stamp=m.header.stamp.to_sec()
 with lock:
  recent.append(m)
  for key,d in list(active.items()):
   floor=d['request']['capture_after']
   if floor<=stamp<=floor+1.:
    name='%s-%s-%.3f.png'%(key[0],key[1],stamp)
    cv2.imwrite(str(args.out/name),bridge.imgmsg_to_cv2(m,'bgr8'))
    d['frames'].append(dict(stamp=stamp,file=name))
def result(m):
 d=json.loads(m.data)
 with lock:
  saved=active.pop((d['mission_id'],d['request_id']),None)
  if saved:
   saved['result']=d
   saved['exact_frame']=next((x['file'] for x in saved['frames'] if abs(x['stamp']-d['capture_stamp'])<.0005),None)
   rows.append(saved)
rospy.Subscriber('/uav/instruction',String,request,queue_size=10)
rospy.Subscriber('/iris_depth_camera/camera/rgb/image_raw',Image,image,queue_size=5)
rospy.Subscriber('/uav/detection_result',String,result,queue_size=10)
while not stop.is_set() and not rospy.is_shutdown():time.sleep(.1)
with lock:(args.out/'observations.json').write_text(json.dumps(dict(rows=rows,pending=list(active.values())),indent=2)+'\n')
