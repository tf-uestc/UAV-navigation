#!/usr/bin/env python3
"""Record conservative target clearance from actual Gazebo poses, without control."""
import argparse,hashlib,json,math,signal,time
from pathlib import Path
import xml.etree.ElementTree as ET
import rospy
from gazebo_msgs.srv import GetLinkState,GetModelState

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
 ap.add_argument('--iris-sdf',type=Path,required=True);args=ap.parse_args()
 rospy.init_node('target_clearance_observer',anonymous=True,disable_signals=True)
 stopped=[False]
 for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *a:stopped.__setitem__(0,True))
 root=ET.parse(str(args.iris_sdf)).getroot();bounds=[]
 def position(e):
  text=e.findtext('pose','0 0 0 0 0 0').split();return [float(x) for x in text[:3]]
 for link in root.findall('.//model/link'):
  center=position(link)
  for c in link.findall('collision'):
   offset=position(c);g=c.find('geometry');radius=None
   if g.find('box') is not None:
    size=[float(x) for x in g.findtext('box/size').split()]
    radius=math.sqrt(sum((x/2)**2 for x in size))
   elif g.find('cylinder') is not None:
    r=float(g.findtext('cylinder/radius'));h=float(g.findtext('cylinder/length'))
    radius=math.hypot(r,h/2)
   elif g.find('sphere') is not None:radius=float(g.findtext('sphere/radius'))
   if radius is not None:bounds.append(math.sqrt(sum((a+b)**2 for a,b in zip(center,offset)))+radius)
 assert bounds
 vehicle_radius=max(bounds)
 world=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
 model=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
 samples=[]
 try:
  while not stopped[0] and not rospy.is_shutdown():
   a=world('iris_depth_camera::iris::base_link','world');b=model('red_tripod_A','world')
   if a.success and b.success:
    p=a.link_state.pose.position;t=b.pose.position
    distance=math.hypot(p.x-t.x,p.y-t.y)
    samples.append(dict(stamp=rospy.Time.now().to_sec(),world_xyz=[p.x,p.y,p.z],
       target_xyz=[t.x,t.y,t.z],horizontal_distance_m=distance,
       conservative_clearance_m=distance-.05-vehicle_radius))
   time.sleep(.10)
 finally:
  args.out.parent.mkdir(parents=True,exist_ok=True)
  report=dict(samples=samples,count=len(samples),vehicle_bounding_radius_m=vehicle_radius,
    model_sha256=hashlib.sha256(args.iris_sdf.read_bytes()).hexdigest(),
    minimum_horizontal_distance_m=min((x['horizontal_distance_m'] for x in samples),default=None),
    minimum_conservative_clearance_m=min((x['conservative_clearance_m'] for x in samples),default=None))
  args.out.write_text(json.dumps(report,indent=2)+'\n')
  print(json.dumps({k:v for k,v in report.items() if k!='samples'}))
if __name__=='__main__':main()
