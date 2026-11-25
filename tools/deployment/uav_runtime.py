#!/usr/bin/python3
"""Own and restore the pinned project SITL without automatically arming it."""
import argparse,json,math,os,re,signal,socket,stat,subprocess,sys,time,uuid
from pathlib import Path
class Runtime:
 def __init__(self):
  self.root=Path(os.environ['UAV_RUNTIME_HOME']).resolve()
  self.project=Path(os.environ['UAV_PROJECT_DIR']).resolve()
  self.state=self.root/'session-state';self.state.mkdir(parents=True,exist_ok=True)
  self.initialized=False
 def token(self,pid):
  try:
   fields=Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
   return None if fields[0]=='Z' else fields[19]
  except FileNotFoundError:return None
 def owner(self,role):
  path=self.state/(role+'.json')
  if not path.exists():return None
  record=json.loads(path.read_text());pid=int(record['pid'])
  if self.token(pid)!=record['start_ticks']:return None
  command=Path('/proc/%d/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode()
  if record['marker'] not in command:raise RuntimeError('PID identity mismatch for '+role)
  return record
 def remember(self,role,proc,marker,log):
  time.sleep(.2)
  if proc.poll() is not None:raise RuntimeError(role+' exited; inspect '+str(log))
  record=dict(pid=proc.pid,start_ticks=self.token(proc.pid),marker=marker,log=str(log))
  (self.state/(role+'.json')).write_text(json.dumps(record,indent=2)+'\n')
 def master(self):
  try:
   with socket.create_connection(('127.0.0.1',11311),timeout=.3):return True
  except OSError:return False
 def ros(self):
  if not self.master():raise RuntimeError('SITL ROS master is not running')
  import rospy
  if not self.initialized:
   rospy.init_node('uav_runtime_client',anonymous=True,disable_signals=True,log_level=rospy.ERROR)
   self.initialized=True
  return rospy
 def health(self,camera=False):
  rospy=self.ros()
  from mavros_msgs.msg import State,ExtendedState
  from gazebo_msgs.srv import GetLinkState
  assert rospy.get_param('/use_sim_time',False) is True,'not a simulated clock'
  assert rospy.get_param('/mavros/fcu_url','')=='udp://:14540@127.0.0.1:14580','unexpected FCU endpoint'
  state=rospy.wait_for_message('/mavros/state',State,timeout=3.)
  landed=rospy.wait_for_message('/mavros/extended_state',ExtendedState,timeout=3.)
  assert state.connected,'MAVROS is disconnected'
  assert rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)('iris_depth_camera::iris::base_link','world').success
  if camera:
   from sensor_msgs.msg import Image
   image=rospy.wait_for_message('/iris_depth_camera/camera/rgb/image_raw',Image,timeout=3.)
   assert image.width==640 and image.height==480 and image.header.stamp.to_sec()>0
  return dict(connected=state.connected,armed=state.armed,landed=landed.landed_state==1,mode=state.mode)
 def check(self):
  import cv2,numpy,rospy
  found={}
  for package,key in [('ego_planner','UAV_EGO_INSTALL'),('uav_vln_bringup','UAV_BUSINESS_INSTALL')]:
   path=subprocess.check_output(['/opt/ros/noetic/bin/rospack','find',package],text=True).strip()
   expected=Path(os.environ[key])/'share'/package
   assert Path(path).resolve()==expected.resolve(),(package,path,str(expected))
   found[package]=path
  assert sys.executable=='/usr/bin/python3'
  assert numpy.__file__.startswith('/usr/lib/python3/dist-packages/')
  px4=Path(os.environ['PX4_AUTOPILOT_DIR'])
  assert (px4/'build/px4_sitl_default/bin/px4').is_file()
  commit=subprocess.check_output(['git','-C',str(px4),'rev-parse','HEAD'],text=True).strip()
  assert commit=='b8c541dd7277ed735139d7d1bfb829d61fbe29fb'
  return dict(passed=True,python=sys.executable,numpy=numpy.__version__,numpy_file=numpy.__file__,
   opencv=cv2.__version__,opencv_file=cv2.__file__,packages=found,px4_commit=commit)
 def vnc(self):
  def active():
   try:
    with socket.create_connection(('127.0.0.1',6006),timeout=.6) as s:return s.recv(12).startswith(b'RFB ')
   except OSError:return False
  if active():return dict(running=True,display=':1',port=6006)
  for path in Path('/proc').glob('[0-9]*/cmdline'):
   try:cmd=path.read_bytes().replace(b'\0',b' ').decode(errors='replace')
   except (OSError,PermissionError):continue
   if re.search(r'\b(Xvnc|Xorg)\b.*\s:1(?:\s|$)',cmd):
    raise RuntimeError('Display :1 exists but its VNC listener is unavailable')
  unix=Path('/tmp/.X11-unix/X1')
  if unix.exists():
   s=socket.socket(socket.AF_UNIX);s.settimeout(.3)
   try:s.connect(str(unix));raise RuntimeError('Display :1 Unix socket is live')
   except (ConnectionRefusedError,FileNotFoundError):pass
   finally:s.close()
  for path in (Path('/tmp/.X1-lock'),unix):
   if path.exists():
    mode=path.lstat().st_mode
    assert not stat.S_ISLNK(mode) and (stat.S_ISREG(mode) or stat.S_ISSOCK(mode))
    path.rename(path.with_name(path.name+'.uav-stale-'+str(time.time_ns())))
  assert any(p.exists() for p in (Path('/root/.vnc/passwd'),Path('/root/.config/turbovnc/passwd'))),'VNC password file is missing'
  subprocess.run(['/opt/TurboVNC/bin/vncserver',':1','-wm','xfce','-geometry','1600x900','-depth','24',
                  '-localhost','-rfbport','6006'],check=True,stdout=subprocess.DEVNULL,timeout=25.)
  for _ in range(30):
   if active():return dict(running=True,display=':1',port=6006)
   time.sleep(.2)
  raise RuntimeError('VNC listener did not start')
 def start(self):
  self.check()
  if self.owner('sim'):
   return dict(reused=True,state=self.health(camera=True),owner=self.owner('sim'))
  if self.master() or subprocess.run(['pgrep','-x','px4'],stdout=subprocess.DEVNULL).returncode==0 or subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode==0:
   raise RuntimeError('An unowned ROS/PX4/Gazebo process is running; refusing a duplicate')
  self.vnc()
  out=self.root/'logs'/('sim-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir(parents=True)
  env=os.environ.copy();env['UAV_RUN_DIR']=str(out)
  with (out/'supervisor.log').open('w') as sink:
   proc=subprocess.Popen(['bash',str(self.project/'tools/deployment/launch_project_rgbd.sh')],
      cwd='/tmp',env=env,stdout=sink,stderr=subprocess.STDOUT,start_new_session=True)
  self.remember('sim',proc,'launch_project_rgbd.sh',out/'supervisor.log')
  last=''
  deadline=time.monotonic()+100.
  while time.monotonic()<deadline:
   if not self.owner('sim'):raise RuntimeError('SITL supervisor exited; inspect '+str(out))
   try:return dict(reused=False,state=self.health(camera=True),owner=self.owner('sim'))
   except Exception as exc:last=str(exc);time.sleep(1.)
  raise RuntimeError('SITL startup timed out: '+last)
 def session(self,height,standoff):
  self.check();state=self.health(camera=True)
  if self.owner('mission'):
   import rospy
   assert abs(rospy.get_param('/segment_mission/height')-height)<1e-8
   assert abs(rospy.get_param('/segment_mission/standoff')-standoff)<1e-8
   return dict(reused=True,owner=self.owner('mission'))
  assert not state['armed'] and state['landed'],'start a new controller only while landed and disarmed'
  assert 1.1<=height<=2.35 and math.isfinite(standoff) and standoff>0
  out=self.root/'logs'/('mission-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir(parents=True)
  cmd=['/opt/ros/noetic/bin/roslaunch','uav_vln_bringup','segment_verified.launch','require_known_space:=true',
       'visual_takeoff_height:=1.6','require_sitl_heading:=true',
       'height:='+str(height),'standoff:='+str(standoff)]
  with (out/'launch.log').open('w') as sink:
   proc=subprocess.Popen(cmd,cwd='/tmp',stdout=sink,stderr=subprocess.STDOUT,start_new_session=True)
  self.remember('mission',proc,'segment_verified.launch',out/'launch.log')
  from std_msgs.msg import String
  rospy=self.ros()
  status=json.loads(rospy.wait_for_message('/uav/segment/status',String,timeout=20.).data)
  assert status['phase']=='IDLE',status
  expires=time.monotonic()+18.;flight={}
  while time.monotonic()<expires:
   flight=json.loads(rospy.wait_for_message('/uav/flight/status',String,timeout=3.).data)
   if flight.get('ready'):break
   time.sleep(.1)
  assert flight.get('ready'),'controller graph not ready'
  return dict(reused=False,owner=self.owner('mission'),mission_status=status)
 def send(self,text,takeoff,dry,land=False):
  rospy=self.ros()
  from std_msgs.msg import String
  status=json.loads(rospy.wait_for_message('/uav/segment/status',String,timeout=5.).data)
  command=dict(command='LAND' if land else 'START',mission_session_id=status['mission_session_id'])
  if not land:
   assert status['phase'] in ('IDLE','DONE','FAILED','LANDED'),'mission is active'
   assert text and text.strip(),'target text is required'
   command.update(mission_id='cli-'+uuid.uuid4().hex[:12],target=text,takeoff=takeoff)
  if not dry:
   self.health()
   pub=rospy.Publisher('/uav/segment/command',String,queue_size=1)
   for _ in range(40):
    if pub.get_num_connections():break
    time.sleep(.05)
   assert pub.get_num_connections(),'mission command has no subscriber'
   pub.publish(String(data=json.dumps(command,ensure_ascii=False)));time.sleep(.2)
  return dict(dry_run=dry,command=command)
 def stop_role(self,role):
  record=self.owner(role)
  if not record:return
  os.kill(record['pid'],signal.SIGTERM)
  for _ in range(160):
   if not self.owner(role):return
   time.sleep(.2)
  raise RuntimeError(role+' did not stop; refusing an unverified forced termination')
 def stop(self):
  sim=self.owner('sim');mission=self.owner('mission')
  if not sim and not mission:return dict(stopped=True,reused=True)
  state=self.health();assert not state['armed'] and state['landed'],'land and disarm before stopping'
  self.stop_role('mission');self.stop_role('sim')
  return dict(stopped=True)
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('action',choices=('check','status','vnc','start','session','send','land','stop','restart'))
 p.add_argument('--height',type=float,default=1.6);p.add_argument('--standoff',type=float,default=2.4)
 p.add_argument('--text');p.add_argument('--takeoff',action='store_true');p.add_argument('--dry-run',action='store_true')
 a=p.parse_args();r=Runtime()
 if a.action=='check':out=r.check()
 elif a.action=='status':out=dict(sim=r.owner('sim'),mission=r.owner('mission'),state=r.health() if r.master() else None)
 elif a.action=='vnc':out=r.vnc()
 elif a.action=='start':out=r.start()
 elif a.action=='session':out=r.session(a.height,a.standoff)
 elif a.action in ('send','land'):out=r.send(a.text,a.takeoff,a.dry_run,a.action=='land')
 elif a.action=='stop':out=r.stop()
 else:r.stop();out=r.start()
 print(json.dumps(out,ensure_ascii=False,indent=2),flush=True)
if __name__=='__main__':
 try:main()
 except Exception as exc:print(json.dumps(dict(passed=False,error=str(exc))),flush=True);raise SystemExit(1)
