#!/usr/bin/env python3
"""Isolated real-ROS tests; starts its own master, never connects to flight master."""
import argparse, atexit, copy, json, math, os, signal, socket, subprocess, sys, threading, time, uuid
from pathlib import Path
import xmlrpc.client
import numpy as np

ap=argparse.ArgumentParser()
ap.add_argument('--stage',choices=['lifecycle','ego'],required=True)
ap.add_argument('--clock',choices=['wall','sim'],default='wall')
ap.add_argument('--out',required=True)
ap.add_argument('--oracle',default='/root/autodl-tmp/uav/records/step78-2026-11-02/bspline_oracle')
args=ap.parse_args()
out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
port=11312 if args.stage=='lifecycle' else 11313
os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%port
os.environ['ROS_IP']='127.0.0.1'
sock=socket.socket(); busy=sock.connect_ex(('127.0.0.1',port))==0; sock.close()
if busy: raise RuntimeError('Isolated test port already occupied; refusing reuse')
processes=[]
logs=[]

def launch(label,cmd):
    f=(out/(label+'.log')).open('w'); logs.append(f)
    p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    p.stage_log_path=out/(label+'.log')
    processes.append(p)
    return p

def stop(p):
    if p.poll() is not None: return
    os.killpg(p.pid,signal.SIGINT)
    try: p.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=2)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

def wait(pred,timeout=8):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        value=pred()
        if (value.size>0 if isinstance(value,np.ndarray) else bool(value)):return value
        time.sleep(.01)
    raise AssertionError('condition timed out after %.1fs'%timeout)

master=launch('master',['roscore','-p',str(port)])
atexit.register(lambda:[stop(p) for p in reversed(processes)])
for attempt in range(100):
    try:
        xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/stage78_probe')
        break
    except OSError: time.sleep(.1)
else: raise RuntimeError('master did not start')

import rospy
import tf2_ros
from ego_planner.msg import Bspline,GoalCommand,GoalStatus
from geometry_msgs.msg import Point,PoseStamped,TransformStamped,Vector3Stamped
from nav_msgs.msg import Odometry,Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from std_msgs.msg import Header,String
from tf.transformations import quaternion_from_euler,quaternion_matrix,quaternion_from_matrix
from navigation_core.bspline import BSpline
from navigation_core.contracts import DetectionResult,RequestKey

rospy.set_param('/use_sim_time',args.clock=='sim')
rospy.init_node('stage78_test',anonymous=True,disable_signals=True)
events={}
event_lock=threading.Lock()

def collect(topic,typ):
    def callback(m):
        with event_lock:events.setdefault(topic,[]).append((time.monotonic(),m))
    return rospy.Subscriber(topic,typ,callback,queue_size=300)

subscribers=[collect(t,k) for t,k in [('/uav/instruction',String),('/uav/detection_result',String),('/uav/mission_status',String),
 ('/uav/planning/readiness',String),('/uav/planning/result',String),('/uav/planning/request',String),
 ('/uav/planning/trajectory',Bspline),('/uav/ego/command',GoalCommand),('/uav/ego/status',GoalStatus),
 ('/planning/bspline',Bspline),('/grid_map/occupancy',PointCloud2),('/uav/ego/world_velocity',Vector3Stamped)]]

def messages(topic,since=0):return [m for t,m in list(events.get(topic,[])) if t>=since]
def json_messages(topic,since=0):
    result=[]
    for m in messages(topic,since):
        try: result.append(json.loads(m.data))
        except ValueError:pass
    return result

pubs={t:rospy.Publisher(t,typ,queue_size=20,latch=latch) for t,typ,latch in
 [('/uav/instruction',String,False),('/uav/state_cmd',String,False),('/uav/detection_result',String,False),
  ('/uav/planning/request',String,False),('/uav/planning/result',String,False),('/uav/cancel_planning',String,False),
  ('/uav/trajectory',RosPath,True),('/planning/bspline',Bspline,False)]}

class Fixture:
    def __init__(self):
        self.running=True;self.frozen=False;self.ros_time=100.;self.position=np.array([0.,0.,2.])
        self.yaw=0.;self.velocity=np.zeros(3);self.follow=False;self.follow_key=None
        self.input_enabled=True;self.obstacles=[];self.ego_session=uuid.uuid4().hex
        self.map_ready=True;self.auto_reply=True;self.next_traj=1;self.raw=None
        self.clock_pub=rospy.Publisher('/clock',Clock,queue_size=10)
        self.odom_pub=rospy.Publisher('/mavros/local_position/odom',Odometry,queue_size=5)
        self.pose_pub=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=5)
        self.rgb_pub=rospy.Publisher('/iris_depth_camera/camera/rgb/image_raw',Image,queue_size=3)
        self.info_pub=rospy.Publisher('/iris_depth_camera/camera/depth/camera_info',CameraInfo,queue_size=3)
        self.depth_pub=rospy.Publisher('/iris_depth_camera/camera/depth/image_raw',Image,queue_size=3)
        self.grid_depth=rospy.Publisher('/uav/planning/depth',Image,queue_size=3)
        self.grid_pose=rospy.Publisher('/uav/planning/camera_pose',PoseStamped,queue_size=3)
        self.status_pub=rospy.Publisher('/uav/ego/status',GoalStatus,queue_size=20,latch=True)
        self.static=tf2_ros.StaticTransformBroadcaster()
        t=TransformStamped(header=Header(frame_id='map'),child_frame_id='camera_link_optical')
        t.transform.rotation.w=1.;self.static.sendTransform(t)
        self.cmd_sub=rospy.Subscriber('/uav/ego/command',GoalCommand,self.on_command,queue_size=20)
        self.raw_sub=rospy.Subscriber('/uav/planning/trajectory',Bspline,self.on_verified,queue_size=20)
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()

    def fake_status(self,command,status):
        m=GoalStatus(header=Header(stamp=rospy.Time.now(),frame_id='map'),
                     planner_session_id=self.ego_session,map_ready=self.map_ready,status=status)
        if command:m.mission_id,m.goal_id=command.mission_id,command.goal_id
        self.status_pub.publish(m)

    def spline(self,command):
        m=Bspline(order=3,traj_id=self.next_traj,start_time=rospy.Time.now(),
                  planner_session_id=self.ego_session,mission_id=command.mission_id,goal_id=command.goal_id)
        self.next_traj+=1
        m.knots=[-3.,-2.,-1.,0.,1.,2.,3.,4.]
        m.pos_pts=[Point(float(i),0.,2.) for i in range(4)]
        return m

    def on_command(self,m):
        if args.stage!='lifecycle' or not self.auto_reply:return
        if m.command=='CANCEL':self.fake_status(m,'CANCELLED');return
        if not self.map_ready:return
        # Deliberately send trajectory before acceptance on a separate ROS channel.
        raw=self.spline(m);pubs['/planning/bspline'].publish(raw)
        time.sleep(.03);self.fake_status(m,'ACCEPTED')

    def on_verified(self,m):
        if self.follow and (m.mission_id,m.goal_id)==self.follow_key and not m.emergency_stop:
            self.raw=m

    def loop(self):
        tick=0
        while self.running and not rospy.is_shutdown():
            if args.clock=='sim':
                if not self.frozen:self.ros_time+=.02
                self.clock_pub.publish(Clock(clock=rospy.Time.from_sec(self.ros_time)))
            stamp=rospy.Time.now();tick+=1
            if self.follow and self.raw:
                m=self.raw;b=BSpline([(p.x,p.y,p.z) for p in m.pos_pts],m.knots,m.order)
                t=min(b.duration,max(0,(stamp-m.start_time).to_sec()))
                self.position=np.array(b.evaluate(t));self.velocity=np.array(b.derivative().evaluate(t))
            if self.input_enabled:
                q=quaternion_from_euler(0,0,self.yaw)
                h=Header(seq=70000+tick,stamp=stamp,frame_id='map')
                od=Odometry(header=h,child_frame_id='base_link');od.pose.pose.position=Point(*self.position)
                od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z,od.pose.pose.orientation.w=q
                body=quaternion_matrix(q)[:3,:3].T@self.velocity
                od.twist.twist.linear=Point(*body)
                self.odom_pub.publish(od)
                self.pose_pub.publish(PoseStamped(header=h,pose=od.pose.pose))
                if args.stage=='lifecycle' or tick%5==0:self.images(stamp,q)
            if args.stage=='lifecycle' and tick%5==0:self.fake_status(None,'READY' if self.map_ready else 'NOT_READY')
            time.sleep(.02)

    def images(self,stamp,q):
        if args.stage=='lifecycle':
            rgb=Image(header=Header(seq=9123,stamp=stamp,frame_id='camera_link_optical'),height=12,width=16,
                      encoding='rgb8',step=48,data=bytes(16*12*3))
            dep=Image(header=rgb.header,height=12,width=16,encoding='32FC1',step=64,
                      data=np.full((12,16),2.,np.float32).tobytes())
            info=CameraInfo(header=rgb.header,height=12,width=16,K=[10.,0,7.5,0,10.,5.5,0,0,1])
            self.rgb_pub.publish(rgb);self.depth_pub.publish(dep);self.info_pub.publish(info)
            return
        w,h=640,480;fx=343.15907310693535
        v,u=np.mgrid[:h,:w]
        optical=np.stack(((u-319.5)/fx,(v-239.5)/fx,np.ones((h,w))),axis=-1)
        body_from_opt=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        rotation=quaternion_matrix(q)[:3,:3]
        camera=self.position+rotation@np.array([.1,0,0])
        rays=optical@(rotation@body_from_opt).T
        depth=np.full((h,w),5.,dtype=float)
        for center,size in self.obstacles:
            low=np.array(center)-np.array(size)/2;high=np.array(center)+np.array(size)/2
            with np.errstate(divide='ignore',invalid='ignore'):
                a=(low-camera)/rays;b=(high-camera)/rays
                near=np.max(np.minimum(a,b),axis=-1);far=np.min(np.maximum(a,b),axis=-1)
            hit=(far>=np.maximum(near,0))&(near>.2)
            depth=np.where(hit,np.minimum(depth,near),depth)
        header=Header(stamp=stamp,frame_id='camera_link_optical')
        image=Image(header=header,height=h,width=w,encoding='32FC1',step=w*4,data=depth.astype(np.float32).tobytes())
        pose=PoseStamped(header=Header(stamp=stamp,frame_id='map'))
        pose.pose.position=Point(*camera)
        optical_q=quaternion_from_matrix(np.block([[rotation@body_from_opt,np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]]))
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=optical_q
        self.grid_pose.publish(pose);self.grid_depth.publish(image)

fixture=Fixture()
checks=[]

def case(name,fn):
    start=time.monotonic()
    try:
        details=fn()
        checks.append(dict(name=name,passed=True,seconds=time.monotonic()-start,details=details))
        print('PASS',name,flush=True)
    except Exception as e:
        import traceback
        checks.append(dict(name=name,passed=False,seconds=time.monotonic()-start,error=repr(e),trace=traceback.format_exc()))
        print('FAIL',name,repr(e),flush=True)

def readiness():
    ds=json_messages('/uav/planning/readiness')
    return ds[-1] if ds else None

def request(goal=(3.,0.,2.),mission=None,goal_id=1):
    r=wait(readiness)
    d=dict(schema_version=1,mission_id=mission or uuid.uuid4().hex,goal_id=goal_id,
           gateway_session_id=r['gateway_session_id'],frame_id='map',goal=list(goal))
    pubs['/uav/planning/request'].publish(String(data=json.dumps(d)))
    return d

def result_for(d,status=None,since=0):
    return next((x for x in reversed(json_messages('/uav/planning/result',since))
                 if x.get('mission_id')==d['mission_id'] and x.get('goal_id')==d['goal_id']
                 and (status is None or x.get('status')==status)),None)

def raw_for(d,topic='/uav/planning/trajectory'):
    return [m for m in messages(topic) if m.mission_id==d['mission_id'] and m.goal_id==d['goal_id']]

def gateway():return launch('gateway-%d'%len(processes),['rosrun','uav_vln_bringup','managed_planner_node.py','_plan_timeout:=0.5' if args.stage=='lifecycle' else '_plan_timeout:=8.0'])

manager=None
def start_manager():
    return launch('manager-%d'%len(processes),['rosrun','uav_vln_bringup','mission_manager.py',
        '_identified_planning:=true','_plan_timeout:=0.4','_plan_retries:=1',
        '_search_vlm_timeout:=5.0','_approach_vlm_timeout:=0.5','_search_vlm_interval:=0.1',
        '_approach_vlm_interval:=100.0'])

def mission(target='red'):
    pubs['/uav/state_cmd'].publish(String(data='HOVER'));time.sleep(.1)
    since=time.monotonic()
    pubs['/uav/instruction'].publish(String(data=json.dumps({'target':{'text':target}})))
    d=wait(lambda:next((d for d in json_messages('/uav/planning/request',since)),None),12)
    return d,since

fatal=None
try:
    if args.stage=='lifecycle':
        gp=gateway()
        detector=launch('detector',['rosrun','uav_vln_bringup','detector_node.py',
                '_detector_class:=test_backend.FixtureDetector','_managed_requests_only:=true'])
        manager=start_manager()
        wait(lambda:readiness() and readiness()['ready'])
        wait(lambda:pubs['/uav/instruction'].get_num_connections()>=2)
        time.sleep(.3)

        def transport():
            start=time.monotonic();m=uuid.uuid4().hex
            for request_id in (1042,9007):
                pubs['/uav/instruction'].publish(String(data=json.dumps(dict(target={'text':'slow_transport'},
                    mission_id=m,request_id=request_id,_mission_mgr_trigger=True))))
                time.sleep(.025)
            rs=wait(lambda:[r for r in json_messages('/uav/detection_result',start) if r['mission_id']==m and r['status']=='SUCCEEDED'])
            all_results=[r for r in json_messages('/uav/detection_result',start) if r['mission_id']==m]
            assert any(r['request_id']==1042 and r['status']=='SUCCEEDED' for r in all_results)
            assert any(r['request_id']==9007 and r['status']=='BUSY' for r in all_results)
            time.sleep(.08)
            pubs['/uav/instruction'].publish(String(data=json.dumps(dict(target={'text':'missing'},
                mission_id=m,request_id=2222,_mission_mgr_trigger=True))))
            wait(lambda:any(r['mission_id']==m and r['request_id']==2222 and r['status']=='FAILED'
                 for r in json_messages('/uav/detection_result',start)))
            return all_results
        case('real_detector_ids_and_fast_busy',transport)

        def early_spline():
            d=request();wait(lambda:result_for(d,'OK'))
            assert len(raw_for(d))==1
            return d
        case('spline_before_ack_is_buffered_then_forwarded',early_spline)

        def cancel_late():
            d=request();wait(lambda:result_for(d,'OK'));m=raw_for(d)[-1]
            pubs['/uav/cancel_planning'].publish(String(data='cancel'))
            wait(lambda:result_for(d,'CANCELLED'));n=len(raw_for(d))
            pubs['/planning/bspline'].publish(m);time.sleep(.2)
            assert len(raw_for(d))==n
        case('cancel_rejects_late_spline',cancel_late)

        def reordered_cancel():
            mission_id=uuid.uuid4().hex;r=readiness()
            cancellation=dict(schema_version=1,gateway_session_id=r['gateway_session_id'],
                              mission_id=mission_id,through_goal_id=1)
            pubs['/uav/cancel_planning'].publish(String(data=json.dumps(cancellation)));time.sleep(.08)
            d=request(mission=mission_id)
            wait(lambda:result_for(d,'REJECTED'))
            assert not [m for m in messages('/uav/ego/command') if m.mission_id==mission_id and m.command=='GOAL']
            fresh=request();wait(lambda:result_for(fresh,'OK'));start=time.monotonic()
            pubs['/uav/cancel_planning'].publish(String(data=json.dumps(cancellation)));time.sleep(.15)
            assert not [m for m in messages('/uav/ego/command',start) if m.mission_id==fresh['mission_id'] and m.command=='CANCEL']
        case('cancel_before_request_and_delayed_old_cancel_are_isolated',reordered_cancel)

        def ready_retry():
            fixture.map_ready=False;wait(lambda:not readiness()['ready'])
            d=request();wait(lambda:result_for(d,'MAP_NOT_READY'))
            fixture.map_ready=True;wait(lambda:readiness()['ready'])
            pubs['/uav/planning/request'].publish(String(data=json.dumps(d)))
            wait(lambda:result_for(d,'OK'))
        case('map_not_ready_then_ready_same_request',ready_retry)

        def timeout_cancel():
            fixture.auto_reply=False
            d=request();wait(lambda:result_for(d,'FAILED'))
            wait(lambda:any(m.command=='CANCEL' and m.mission_id==d['mission_id'] for m in messages('/uav/ego/command')),.3)
            fixture.auto_reply=True
        case('bounded_planner_timeout_sends_cancel',timeout_cancel)

        def wrong_old_path():
            fixture.map_ready=False;fixture.auto_reply=False
            d,start=mission()
            stale=dict(schema_version=1,mission_id='wrong-task',goal_id=d['goal_id'],status='OK',gateway_session_id=d['gateway_session_id'])
            pubs['/uav/planning/result'].publish(String(data=json.dumps(stale)))
            path=RosPath(header=Header(stamp=rospy.Time.now(),frame_id='map'),poses=[PoseStamped()])
            pubs['/uav/trajectory'].publish(path)
            wait(lambda:any(m.data=='HOVER' for m in messages('/uav/mission_status',start)),2)
            fixture.map_ready=True;fixture.auto_reply=True
            return 'Old task and unidentified latched Path could not complete plan wait.'
        case('manager_rejects_wrong_task_and_latched_path',wrong_old_path)

        def permanent_not_ready():
            fixture.map_ready=False;wait(lambda:not readiness()['ready'])
            d,start=mission()
            wait(lambda:any(m.data=='HOVER' for m in messages('/uav/mission_status',start)),2)
            requests=[x for x in json_messages('/uav/planning/request',start) if x['mission_id']==d['mission_id']]
            assert len(requests)<=2,len(requests)
            fixture.map_ready=True
            return dict(requests=len(requests))
        case('manager_permanent_not_ready_has_bounded_retries',permanent_not_ready)

        def task_switch():
            d,start=mission();wait(lambda:result_for(d,'OK'))
            new,changed=mission('blue');wait(lambda:result_for(new,'OK'))
            assert d['mission_id']!=new['mission_id']
            pubs['/uav/planning/result'].publish(String(data=json.dumps(dict(schema_version=1,
                mission_id=d['mission_id'],goal_id=d['goal_id'],gateway_session_id=d['gateway_session_id'],status='FAILED'))))
            time.sleep(.15)
            assert not any(m.data=='HOVER' for m in messages('/uav/mission_status',changed+.5))
        case('task_A_to_B_old_failure_is_ignored',task_switch)

        def ego_restart():
            d=request();wait(lambda:result_for(d,'OK'));old=raw_for(d)[-1]
            fixture.auto_reply=False;fixture.ego_session=uuid.uuid4().hex
            wait(lambda:result_for(d,'SESSION_CHANGED'))
            n=len(raw_for(d));pubs['/planning/bspline'].publish(old);time.sleep(.2)
            assert len(raw_for(d))==n
            fixture.auto_reply=True
        case('ego_restart_session_does_not_replay_active_goal',ego_restart)

        def gateway_restart():
            global gp
            old=readiness()['gateway_session_id']
            d=request();wait(lambda:result_for(d,'OK'))
            latch=rospy.Publisher('/uav/planning/request',String,queue_size=2,latch=True)
            time.sleep(.1);latch.publish(String(data=json.dumps(d)))
            stop(gp);gp=gateway()
            wait(lambda:readiness()['gateway_session_id']!=old)
            time.sleep(.3)
            # A newly constructed publisher may not yet have a result subscriber
            # when the latched request is delivered; retransmit to inspect rejection.
            pubs['/uav/planning/request'].publish(String(data=json.dumps(d)))
            wait(lambda:result_for(d,'REJECTED'))
            latch.unregister()
            return 'Old latched request rejected by new gateway session.'
        case('gateway_restart_rejects_latched_old_request',gateway_restart)

        def manager_restart_ids():
            global manager
            previous=next(d for d in reversed(json_messages('/uav/instruction')) if d.get('_mission_mgr_trigger'))
            stop(manager)
            wait(lambda:pubs['/uav/state_cmd'].get_num_connections()==0)
            manager=start_manager()
            wait(lambda:pubs['/uav/state_cmd'].get_num_connections()==1)
            # One acknowledged command; repeated HOVERs on another topic could
            # arrive after the subsequent instruction and invalidate the test.
            ready_at=time.monotonic()
            pubs['/uav/state_cmd'].publish(String(data='HOVER'))
            wait(lambda:any(m.data=='HOVER' for m in messages('/uav/mission_status',ready_at)))
            start=time.monotonic()
            pubs['/uav/instruction'].publish(String(data=json.dumps({'target':{'text':'slow_restart'}})))
            current=wait(lambda:next((d for d in json_messages('/uav/instruction',start) if d.get('_mission_mgr_trigger')),None),8)
            assert current['request_id']==1
            assert current['mission_id']!=previous['mission_id']
            stamp=rospy.Time.now().to_sec()
            wrong=DetectionResult(RequestKey(previous['mission_id'],current['request_id']),
                   'slow_restart','SUCCEEDED',stamp,stamp,(10,0,2))
            pubs['/uav/detection_result'].publish(String(data=wrong.to_json()))
            time.sleep(.2)
            assert not [d for d in json_messages('/uav/planning/request',start) if d['mission_id']==current['mission_id']]
            time.sleep(.5)
        case('manager_restart_reused_request_number_rejects_old_mission',manager_restart_ids)

        def manager_fast_busy():
            pubs['/uav/state_cmd'].publish(String(data='HOVER'));time.sleep(.15)
            start=time.monotonic()
            pubs['/uav/instruction'].publish(String(data=json.dumps({'target':{'text':'busy_test'}})))
            time.sleep(3.15)
            pubs['/uav/instruction'].publish(String(data=json.dumps(dict(target={'text':'slow_occupier'},
                mission_id=uuid.uuid4().hex,request_id=9999,_mission_mgr_trigger=True))))
            requests=lambda:[d for d in json_messages('/uav/instruction',start)
                              if d.get('_mission_mgr_trigger') and d.get('target',{}).get('text')=='busy_test']
            first=wait(lambda:requests() and requests()[0],5)
            wait(lambda:any(r['mission_id']==first['mission_id'] and r['request_id']==first['request_id'] and r['status']=='BUSY'
                            for r in json_messages('/uav/detection_result',start)),2)
            busy_wall=time.monotonic()
            wait(lambda:len(requests())>=2,3)
            elapsed=time.monotonic()-busy_wall
            assert elapsed<3  # configured request timeout is 5 s, so BUSY was resolved immediately.
            return dict(seconds_to_next_request=elapsed,configured_timeout=5)
        case('manager_fast_busy_clears_pending_before_timeout',manager_fast_busy)

        if args.clock=='sim':
            def paused_clock():
                fixture.map_ready=False;wait(lambda:not readiness()['ready'])
                d,start=mission();fixture.frozen=True;t=time.monotonic()
                wait(lambda:any(m.data=='HOVER' for m in messages('/uav/mission_status',start)),1.4)
                elapsed=time.monotonic()-t
                fixture.frozen=False;fixture.map_ready=True
                return dict(wall_seconds_until_hover=elapsed)
            case('paused_ros_clock_keeps_wall_deadlines',paused_clock)
    else:
        stack=launch('ego-stack',['roslaunch','uav_vln_bringup','planner_verified.launch','with_camera_bridge:=false'])
        wait(lambda:readiness() and readiness()['ready'],15)

        def cloud_xyz(m):
            offsets=[next(f.offset for f in m.fields if f.name==name) for name in ['x','y','z']]
            dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':offsets,'itemsize':m.point_step})
            a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
            return np.stack([a[k].reshape(-1) for k in ['x','y','z']],axis=-1)

        def cancel():
            pubs['/uav/cancel_planning'].publish(String(data='test_cancel'))
            fixture.follow=False;fixture.raw=None;fixture.velocity=np.zeros(3)
            time.sleep(.15)

        def spline_data(d):
            wait(lambda:raw_for(d),10)
            return raw_for(d)[-1]

        def sample(m):return BSpline([(p.x,p.y,p.z) for p in m.pos_pts],m.knots,m.order)

        def velocity_axes():
            errors=[]
            for angle in [0,math.pi/2,math.pi,-math.pi/2]:
                fixture.yaw=angle;fixture.velocity=np.array([math.cos(angle),math.sin(angle),0.])
                start=time.monotonic()
                wait(lambda:len(messages('/uav/ego/world_velocity',start))>=4)
                values=messages('/uav/ego/world_velocity',start)[-1].vector
                err=np.linalg.norm(np.array([values.x,values.y,values.z])-fixture.velocity)
                assert err<1e-6,err
                errors.append(float(err))
            fixture.yaw=0.;fixture.velocity=np.zeros(3)
            return errors
        case('body_velocity_rotates_into_world_at_four_yaws',velocity_axes)

        def map_geometry():
            fixture.obstacles=[([2.,0.,2.],[.6,.8,1.])]
            start=time.monotonic()
            def observed():
                ms=messages('/grid_map/occupancy',start)
                if not ms:return False
                pts=cloud_xyz(ms[-1]);roi=pts[(pts[:,0]>1.4)&(pts[:,0]<2.5)&(abs(pts[:,1])<.7)&(pts[:,2]>1.4)&(pts[:,2]<2.5)]
                return roi if len(roi)>20 else None
            pts=wait(observed,8)
            assert abs(np.min(pts[:,0])-1.7)<.16
            master_api=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
            state=master_api.getSystemState('/stage78_test')[2]
            assert not [x for x in state[1] if x[0]=='/grid_map/cloud']
            return dict(occupied_roi_count=len(pts),nearest_x=float(np.min(pts[:,0])))
        case('known_obstacle_is_in_world_map_single_depth_input',map_geometry)

        def rotated_map():
            fixture.position=np.array([.2,.1,2.]);fixture.yaw=math.pi/2
            fixture.obstacles.append(([0.,2.,2.],[.8,.6,1.]))
            start=time.monotonic()
            def observed():
                ms=messages('/grid_map/occupancy',start)
                if not ms:return None
                pts=cloud_xyz(ms[-1]);roi=pts[(abs(pts[:,0])<.7)&(pts[:,1]>1.4)&(pts[:,1]<2.5)&(pts[:,2]>1.4)&(pts[:,2]<2.5)]
                return roi if len(roi)>20 else None
            try:
                pts=wait(observed,8)
                assert abs(np.min(pts[:,1])-1.7)<.16
                result=dict(occupied_roi_count=len(pts),nearest_y=float(np.min(pts[:,1])))
            finally:
                fixture.position=np.array([0.,0.,2.]);fixture.yaw=0.
                fixture.obstacles=[([2.,0.,2.],[.6,.8,1.])]
                time.sleep(.7)
            return result
        case('camera_translation_and_90deg_yaw_keep_obstacle_in_world',rotated_map)

        def invalid_goal():
            # Depth observes the front surface, not the unknown interior of a box.
            for g in [(1.75,.05,2.05),(100,0,2),(2,2,2.7)]:
                d=request(g);wait(lambda:result_for(d,'REJECTED'))
                assert not raw_for(d)
        case('occupied_out_of_bounds_and_ceiling_goals_rejected',invalid_goal)

        def heights():
            data=[]
            for z in [1.5,2.,2.2]:
                d=request((3.,1.8,z));m=spline_data(d);b=sample(m)
                endpoint=b.evaluate(b.duration)
                assert abs(endpoint[2]-z)<.01,(z,endpoint)
                data.append(dict(requested_z=z,actual_z=endpoint[2]))
                cancel()
            return data
        case('requested_target_heights_are_preserved',heights)

        def numerical_oracle():
            m=messages('/uav/planning/trajectory')[-1];b=sample(m)
            values=[str(m.order),str(len(m.pos_pts)),str(len(m.knots)),'101']
            values += [repr(float(v)) for p in m.pos_pts for v in [p.x,p.y,p.z]]
            values += [repr(float(k)) for k in m.knots]
            proc=subprocess.run([args.oracle],input=' '.join(values),text=True,capture_output=True,check=True)
            native=np.array([[float(v) for v in row.split()] for row in proc.stdout.splitlines()])
            velocity=b.derivative();acc=velocity.derivative()
            py=np.array([[t,*b.evaluate(t),*velocity.evaluate(t),*acc.evaluate(t)] for t in native[:,0]])
            error=np.max(np.abs(native-py),axis=0)
            assert max(error)<1e-6,error
            return dict(samples=len(native),max_errors=error.tolist())
        case('python_matches_native_uniform_bspline_at_101_times',numerical_oracle)

        def obstacle_avoidance():
            d=request((4.,0.,2.));m=spline_data(d);b=sample(m)
            points=np.array([b.evaluate(t) for t in np.linspace(0,b.duration,301)])
            inside=(np.abs(points[:,0]-2)<.3)&(np.abs(points[:,1])<.4)&(np.abs(points[:,2]-2)<.5)
            assert not inside.any(),points[inside][:3]
            cancel()
            return dict(samples=301,y_extent=[float(points[:,1].min()),float(points[:,1].max())])
        case('planned_path_avoids_known_obstacle',obstacle_avoidance)

        def continuous():
            cancel();fixture.position=np.array([0.,0.,2.]);fixture.yaw=0.;fixture.obstacles=[]
            time.sleep(.7)
            mission_id=uuid.uuid4().hex;fixture.follow=True;fixture.follow_key=(mission_id,1)
            d=request((6.,2.5,2.),mission=mission_id)
            wait(lambda:len(raw_for(d))>=3,14)
            raws=raw_for(d);ids=[m.traj_id for m in raws]
            assert len(set(ids))==len(ids)
            native=raw_for(d,'/planning/bspline')
            for m in raws:
                n=next(x for x in native if x.traj_id==m.traj_id and x.start_time==m.start_time)
                import io
                a=io.BytesIO();c=io.BytesIO();m.serialize(a);n.serialize(c)
                assert a.getvalue()==c.getvalue()
            cancel()
            return dict(trajectory_ids=ids,preserved_payload=True)
        case('continuous_replans_preserve_every_native_payload',continuous)

        def dynamic_obstacle():
            cancel();fixture.position=np.array([0.,0.,2.]);fixture.obstacles=[]
            time.sleep(.7)
            mission_id=uuid.uuid4().hex;fixture.follow=True;fixture.follow_key=(mission_id,1)
            d=request((6.,2.5,2.),mission=mission_id);m=spline_data(d)
            b=sample(m)
            # Leave enough observation time for probabilistic occupancy fusion;
            # placing an object inside the near cutoff cannot test replanning.
            ahead=b.evaluate(b.duration*.5)
            start=time.monotonic()
            fixture.obstacles=[(list(ahead),[.7,.7,1.])]
            probe=dict(obstacle_center=list(ahead),duration=b.duration,initial_position=fixture.position.tolist())
            try:
                wait(lambda:any(s.mission_id==d['mission_id'] and s.goal_id==d['goal_id'] and
                     s.reason=='collision' for s in messages('/uav/ego/status',start)) or
                     any(x.emergency_stop for x in raw_for(d)),8)
                wait(lambda:len(raw_for(d))>=2 or result_for(d,'STOP') or result_for(d,'FAILED'),2)
                reactions=len(raw_for(d))
            finally:
                probe['statuses']=[dict(status=s.status,reason=s.reason) for s in messages('/uav/ego/status',start) if s.mission_id==d['mission_id']]
                probe['verified_ids']=[s.traj_id for s in raw_for(d)]
                (out/'dynamic-obstacle-probe.json').write_text(json.dumps(probe,indent=2))
                cancel();fixture.obstacles=[]
            return dict(obstacle_center=list(ahead),verified_trajectory_count=reactions)
        case('new_obstacle_triggers_native_safety_replanning_or_stop',dynamic_obstacle)

        def cancellation():
            fixture.position=np.array([0.,0.,2.]);fixture.obstacles=[];time.sleep(.6)
            d=request((4.,1.5,2.));m=spline_data(d)
            start=time.monotonic();cancel()
            wait(lambda:any(x.emergency_stop for x in raw_for(d,'/planning/bspline')),2)
            wait(lambda:result_for(d,'CANCELLED') and result_for(d,'CANCELLED').get('reason')=='ego_acknowledged',2)
            n=len(raw_for(d))
            late=copy.deepcopy(m);late.traj_id+=100000;late.start_time=rospy.Time.now()
            pubs['/planning/bspline'].publish(late);time.sleep(.3)
            assert len(raw_for(d))==n
            assert not [x for x in messages('/planning/bspline',start+.2) if x.mission_id==d['mission_id'] and not x.emergency_stop and x.traj_id!=late.traj_id]
        case('real_ego_cancel_stops_and_late_spline_cannot_revive',cancellation)

        def resume_from_actual():
            fixture.position=np.array([-.4,-.5,1.8]);fixture.velocity=np.zeros(3);time.sleep(.7)
            d=request((3.,1.8,2.2));m=spline_data(d);b=sample(m)
            error=float(np.linalg.norm(np.array(b.evaluate(0))-fixture.position))
            assert error<.02,error
            cancel()
            return dict(start_error_m=error)
        case('resume_uses_measured_position_not_old_reference',resume_from_actual)

        def stale_map():
            fixture.position=np.array([0.,0.,2.]);time.sleep(.6)
            d=request((4.,1.5,2.));spline_data(d)
            start=time.monotonic();fixture.input_enabled=False
            try:
                wait(lambda:result_for(d,'MAP_NOT_READY'),1.4)
                elapsed=time.monotonic()-start
                wait(lambda:any(m.emergency_stop for m in raw_for(d,'/planning/bspline')),1)
            finally:fixture.input_enabled=True
            wait(lambda:readiness()['ready'],3)
            n=len(raw_for(d));time.sleep(.4);assert len(raw_for(d))==n
            return dict(wall_seconds_until_not_ready=elapsed)
        case('lost_depth_odometry_revokes_map_and_does_not_auto_resume',stale_map)

        def real_restart():
            d=request((4.,1.5,2.));m=spline_data(d);old=m.planner_session_id
            api=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).lookupNode('/stage78_test','/ego_planner_node')[2]
            pid=xmlrpc.client.ServerProxy(api).getPid('/stage78_test')[2]
            os.kill(pid,signal.SIGTERM)
            replacement=launch('ego-restarted',['rosrun','ego_planner','ego_planner_node',
               '/odom_world:=/mavros/local_position/odom','/grid_map/depth:=/uav/planning/depth',
               '/grid_map/pose:=/uav/planning/camera_pose'])
            wait(lambda:readiness()['planner_session_id']!=old and readiness()['ready'],12)
            wait(lambda:result_for(d,'SESSION_CHANGED') or result_for(d,'MAP_NOT_READY'))
            n=len(raw_for(d));pubs['/planning/bspline'].publish(m);time.sleep(.3)
            assert len(raw_for(d))==n
            new=request((3.,1.8,2.2));fresh=spline_data(new)
            assert fresh.planner_session_id!=old
            stale=copy.deepcopy(m);stale.mission_id=new['mission_id'];stale.goal_id=new['goal_id'];stale.traj_id=fresh.traj_id
            count=len(raw_for(new));pubs['/planning/bspline'].publish(stale);time.sleep(.2)
            assert len(raw_for(new))==count
            cancel()
            return dict(old_session=old,new_session=fresh.planner_session_id)
        case('real_ego_restart_changes_epoch_and_rejects_old_trajectory',real_restart)
except Exception as e:
    import traceback
    fatal=repr(e);print(traceback.format_exc(),flush=True)
finally:
    fixture.running=False;fixture.thread.join(timeout=2)
    for p in reversed(processes):stop(p)
    for f in logs:f.close()
    result=dict(stage=args.stage,clock=args.clock,passed=not fatal and bool(checks) and all(x['passed'] for x in checks),checks=checks,fatal=fatal,
                flight_master_contacted=False)
    (out/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,ensure_ascii=False),flush=True)
if not result['passed']:raise SystemExit(1)
