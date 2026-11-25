#!/usr/bin/python3
"""Observe Gazebo physics contacts with a distant falling-box positive control."""
import json,re,signal,subprocess,time
from pathlib import Path
import rospy
from gazebo_msgs.srv import SpawnModel,DeleteModel
from geometry_msgs.msg import Pose,Point
class PhysicsContacts:
 def __init__(self,out,obstacle_model="dynamic_test_box"):
  self.obstacle_model=obstacle_model
  self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True);self.proc=None;self.sink=None;self.witness=False
 def pairs(self,text):
  return re.findall(r'collision1:\s*"([^"]+)"\s+collision2:\s*"([^"]+)"',text)
 def start(self):
  assert rospy.get_param('/use_sim_time') is True
  self.started=rospy.Time.now().to_sec()
  self.sink=(self.out/'physics-contacts.txt').open('w')
  self.proc=subprocess.Popen(['stdbuf','-oL','gz','topic','-e','/gazebo/default/physics/contacts'],
                             stdout=self.sink,stderr=subprocess.STDOUT)
  time.sleep(.5)
  pose=Pose(position=Point(-10.,-10.,.55));pose.orientation.w=1.
  geometry='<geometry><box><size>0.1 0.1 0.1</size></box></geometry>'
  inertia='<inertial><mass>0.1</mass><inertia><ixx>0.000167</ixx><iyy>0.000167</iyy><izz>0.000167</izz></inertia></inertial>'
  sdf='<sdf version="1.6"><model name="contact_witness"><link name="body">'+inertia+'<collision name="c">'+geometry+'</collision><visual name="v">'+geometry+'</visual></link></model></sdf>'
  result=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)('contact_witness',sdf,'',pose,'world')
  assert result.success,result.status_message;self.witness=True
  for _ in range(35):
   text=(self.out/'physics-contacts.txt').read_text()
   if any('contact_witness' in x+y and 'ground' in x+y for x,y in self.pairs(text)):return
   time.sleep(.2)
  raise AssertionError('physics contact observer did not see the positive control')
 def finish(self):
  finished=rospy.Time.now().to_sec()
  if self.proc and self.proc.poll() is None:
   self.proc.send_signal(signal.SIGINT)
   try:self.proc.wait(timeout=4.)
   except subprocess.TimeoutExpired:self.proc.terminate();self.proc.wait(timeout=4.)
  if self.sink:self.sink.close()
  path=self.out/'physics-contacts.txt';text=path.read_text() if path.exists() else ''
  pairs=self.pairs(text)
  positive=sum('contact_witness' in x+y and 'ground' in x+y for x,y in pairs)
  bad=[]
  for x,y in pairs:
   iris_x='iris_depth_camera::' in x
   iris_y='iris_depth_camera::' in y
   if iris_x==iris_y:continue
   other=y if iris_x else x
   if not any(name in other.split('::') for name in ('ground_plane','landing_pad')):
    bad.append((x,y))
  stamps=[float(s)+int(ns)*1e-9 for s,ns in re.findall(r'\btime\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}',text)]
  report=dict(obstacle_model=self.obstacle_model,passed=positive>0 and not bad and len(stamps)>=100 and finished-max(stamps,default=-1)<1.,
   positive_control_contacts=positive,iris_obstacle_contacts=bad[:10],physics_messages=len(stamps),
   start_stamp=self.started,end_stamp=finished,last_message_stamp=max(stamps,default=None))
  (self.out/'physics-contacts.json').write_text(json.dumps(report,indent=2)+'\n')
  if self.witness:rospy.ServiceProxy('/gazebo/delete_model',DeleteModel)('contact_witness')
  return report
