#!/usr/bin/env python3
"""Exercise the installed segmented mission with replay or the live detector."""
import argparse
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

import rosgraph
import rospy
from ego_planner.msg import Bspline
from gazebo_msgs.srv import GetLinkState, SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from navigation_core.contracts import DetectionResult, RequestKey
from navigation_core.yaw import angle_delta
from validate_flight_execution import FakeFcu, decode


def yaw_gap(a,b):
    return abs(math.degrees(angle_delta(a,b)))


class Trial:
    def __init__(self,args):
        self.args=args
        self.mock=FakeFcu() if args.mock else None
        self.mission=self.flight=self.odom=self.state=self.landed=None
        self.trace=[]
        self.checks=[]
        self.scenarios={}
        self.requests=[]
        self.last_spline=None
        self.plan_policy='path'
        self.command=rospy.Publisher('/uav/segment/command',String,queue_size=5)
        self.detection=rospy.Publisher('/uav/detection_result',String,queue_size=5)
        rospy.Subscriber('/uav/segment/status',String,self.json_cb,'mission',queue_size=20)
        rospy.Subscriber('/uav/flight/status',String,self.json_cb,'flight',queue_size=20)
        for topic,label in (('/uav/flight/command','flight_command'),('/uav/planning/request','plan_request'),
                            ('/uav/planning/result','plan_result'),('/uav/detection_result','detect_result')):
            rospy.Subscriber(topic,String,self.json_cb,label,queue_size=30)
        rospy.Subscriber('/mavros/state',State,self.fcu_cb,queue_size=10)
        rospy.Subscriber('/mavros/extended_state',ExtendedState,self.landed_cb,queue_size=2)
        rospy.Subscriber('/mavros/local_position/odom',Odometry,self.odom_cb,queue_size=8)
        rospy.Subscriber('/uav/goal_pose',PoseStamped,self.pose_cb,'goal_pose',queue_size=10)
        rospy.Subscriber('/mavros/setpoint_position/local',PoseStamped,self.pose_cb,'setpoint',queue_size=10)
        rospy.Subscriber('/mavros/setpoint_raw/target_local',PositionTarget,self.target_cb,queue_size=10)
        rospy.Subscriber('/uav/planning/trajectory',Bspline,self.spline_cb,queue_size=10)
        if not args.live:
            rospy.Subscriber('/uav/instruction',String,self.detect_cb,queue_size=10)
        if args.mock:
            self.ready=rospy.Publisher('/uav/planning/readiness',String,queue_size=2)
            self.plan=rospy.Publisher('/uav/planning/result',String,queue_size=3)
            self.spline=rospy.Publisher('/uav/planning/trajectory',Bspline,queue_size=3)
            rospy.Subscriber('/uav/planning/request',String,self.fake_plan,queue_size=5)
            threading.Thread(target=self.heartbeat,daemon=True).start()

    def record(self,label,data):
        self.trace.append((time.monotonic(),label,data))

    def json_cb(self,msg,label):
        data=json.loads(msg.data)
        if label in ('mission','flight'):
            setattr(self,label,data)
        self.record(label,data)

    def fcu_cb(self,msg):
        self.state=msg
        self.record('fcu',dict(mode=msg.mode,armed=msg.armed,connected=msg.connected))

    def landed_cb(self,msg):
        self.landed=msg

    def odom_cb(self,msg):
        self.odom=msg
        p,y=decode(msg.pose.pose)
        v=msg.twist.twist.linear
        self.record('odom',dict(position=p,yaw=y,speed=math.hypot(v.x,v.y,v.z),
                               stamp=msg.header.stamp.to_sec()))

    def pose_cb(self,msg,label):
        p,y=decode(msg.pose)
        self.record(label,dict(position=p,yaw=y,stamp=msg.header.stamp.to_sec()))

    def target_cb(self,msg):
        if not msg.type_mask & 1024:
            self.record('px4_target',dict(yaw=msg.yaw,stamp=msg.header.stamp.to_sec()))

    def spline_cb(self,msg):
        self.record('spline',dict(mission_id=msg.mission_id,goal_id=msg.goal_id,traj_id=msg.traj_id,
                                 emergency=msg.emergency_stop))

    def heartbeat(self):
        while not rospy.is_shutdown():
            try:
                self.ready.publish(String(data=json.dumps(dict(schema_version=1,ready=True,
                    gateway_session_id='replay-gateway',planner_session_id='replay-planner'))))
            except rospy.ROSException:
                if not rospy.is_shutdown():
                    raise
            time.sleep(.05)

    def fake_plan(self,msg):
        data=json.loads(msg.data)
        start,_=decode(self.odom.pose.pose)
        goal=tuple(data['goal'])
        status=dict(schema_version=1,mission_id=data['mission_id'],goal_id=data['goal_id'],
                    gateway_session_id='replay-gateway',planner_session_id='replay-planner')
        if self.plan_policy=='no_route':
            self.plan.publish(String(data=json.dumps(dict(status,status='REJECTED',reason='fixture_no_route'))))
            return
        if self.plan_policy=='outside_fence':
            goal=(7.2,goal[1],goal[2])
        self.last_spline=Bspline(mission_id=data['mission_id'],goal_id=data['goal_id'],planner_session_id='replay-planner',
            traj_id=data['goal_id'],order=3,start_time=rospy.Time.now()+rospy.Duration(.09),
            knots=[0.]*4+[2.]*4,pos_pts=[Point(*p) for p in (start,start,goal,goal)])
        self.spline.publish(self.last_spline)
        self.plan.publish(String(data=json.dumps(dict(status,status='OK'))))

    def detect_cb(self,msg):
        d=json.loads(msg.data)
        mission=d['mission_id']
        scenario=self.scenarios.get(mission)
        if not scenario:
            return
        count=scenario['calls']
        scenario['calls']+=1
        stamp=self.odom.header.stamp.to_sec()
        self.requests.append((time.monotonic(),mission,d['request_id'],stamp,self.flight.get('stop_confirmed')))
        behavior=scenario['behavior']
        if behavior=='silent':
            return
        delay=scenario.get('delay',3.4 if behavior=='late' else .28)
        point=scenario['target']
        if behavior=='jump' and count:
            point=(point[0],point[1]+3.,point[2])
        reason=''
        status='SUCCEEDED'
        if behavior=='missing' or behavior=='search' and count==0:
            status,reason='FAILED','target_not_found'
        elif behavior=='busy':
            status,reason='BUSY','detector_busy'
        elif behavior=='network':
            status,reason='FAILED','api_unavailable'
        if behavior=='stale':
            stamp-=1.
        def reply():
            time.sleep(delay)
            if rospy.is_shutdown():
                return
            result=DetectionResult(RequestKey(mission,d['request_id']),d['target']['text'],
                status,stamp if status!='BUSY' else 0.,rospy.Time.now().to_sec(),
                point if status=='SUCCEEDED' else None,'map',reason)
            self.detection.publish(String(data=result.to_json()))
        threading.Thread(target=reply,daemon=True).start()

    def wait(self,fn,timeout,what):
        end=time.monotonic()+timeout
        while time.monotonic()<end and not rospy.is_shutdown():
            value=fn()
            if value:
                return value
            time.sleep(.025)
        raise AssertionError(what+': '+str(self.mission)+' / '+str(self.flight))

    def check(self,name,**details):
        self.checks.append(dict(name=name,**details))
        print('PASS',name,details,flush=True)

    def send(self,cmd,**payload):
        self.command.publish(String(data=json.dumps(dict(command=cmd,
            mission_session_id=self.mission['mission_session_id'],**payload))))

    def wait_phase(self,mission,phase,seconds):
        return self.wait(lambda: self.mission if self.mission['mission_id']==mission and
            self.mission['phase'] in phase else None,seconds,'mission '+mission+' phase '+str(phase))

    def new(self,name,behavior='normal',takeoff=False,waypoint=None,search=False):
        mission=name+'-'+uuid.uuid4().hex[:8]
        p,y=decode(self.odom.pose.pose)
        direction=(math.cos(y),math.sin(y))
        point=(p[0]+4.*direction[0],p[1]+4.*direction[1],.3)
        self.scenarios[mission]=dict(behavior=behavior,target=point,calls=0)
        fields=dict(mission_id=mission,target=self.args.target,takeoff=takeoff)
        if waypoint is not None:
            fields['waypoint']=waypoint
        if search:
            fields['search_waypoints']=[(p[0]+.6*direction[0],p[1]+.6*direction[1],1.6)]
        self.send('START',**fields)
        return mission

    def stop_and_land(self):
        if self.flight and self.flight.get('owns_flight'):
            self.send('LAND')
            self.wait(lambda: not self.flight.get('owns_flight') and not self.state.armed
                      and self.landed.landed_state==1,40,'landed and disarmed')

    def initial(self):
        self.wait(lambda:self.mission and self.flight and self.flight['ready'] and
                  self.odom and self.state and self.landed and self.command.get_num_connections(),35,'initial ROS')
        assert self.state.connected and not self.state.armed and self.landed.landed_state==1
        if self.args.yaw is not None:
            expect=math.radians(self.args.yaw)
            self.wait(lambda:yaw_gap(decode(self.odom.pose.pose)[1],expect)<12.,85,'estimator initial yaw')
            time.sleep(1.)
        owners=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        assert owners.get('/uav/goal_pose')==owners.get('/mavros/setpoint_position/local')==['/flight_executor']

    def normal(self):
        self.initial()
        name=self.new('nominal',takeoff=True,behavior='search' if self.args.mock else 'normal',
                      search=self.args.mock)
        result=self.wait_phase(name,('DONE','FAILED'),135)
        assert result['phase']=='DONE' and result['segment_count']>=2,result
        phases=[d['phase'] for _,label,d in self.trace if label=='mission' and d['mission_id']==name]
        assert phases.count('TRACK')>=2 and phases.count('WAIT_VLM')>=3,phases
        if not self.args.live:
            checks=[r for r in self.requests if r[1]==name]
            assert len(checks)>=3 and all(r[4] for r in checks),checks
        self.check('segmented_approach_completed',segments=result['segment_count'],
                   phases=phases, mission_id=name)
        end=time.monotonic()
        hold=2. if self.args.mock else 10.
        time.sleep(hold)
        points=[d for t,label,d in self.trace if t>=end and label=='setpoint']
        actual=[d for t,label,d in self.trace if t>=end and label=='odom']
        assert len(points)>18*hold and actual
        max_motion=max(math.dist(d['position'],points[0]['position']) for d in actual)
        max_heading=max(yaw_gap(d['yaw'],points[0]['yaw']) for d in actual)
        assert all(math.dist(d['position'],points[0]['position'])<1e-6 for d in points)
        assert max_motion<=.3 and max_heading<=10.,(max_motion,max_heading)
        self.check('final_hold',seconds=hold,max_position_error_m=max_motion,max_yaw_error_deg=max_heading)
        if self.args.live:
            replies=[d for _,label,d in self.trace if label=='detect_result' and d['mission_id']==name]
            assert 3<=len(replies)<=4 and all(d['status']=='SUCCEEDED' for d in replies),replies
            self.check('bounded_live_calls',count=len(replies))
        if self.args.mock:
            self.check('search_translates_through_planning')
            self.faults()
        self.stop_and_land()
        self.check('landed_disarmed')

    def obstacle(self):
        self.initial()
        link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
        body='iris_depth_camera::iris::base_link'
        ground=link(body,'world')
        assert ground.success
        position=ground.link_state.pose.position
        center=(position.x+.81,position.y)
        size=(.16,.36,2.6)
        geometry=f'<geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>'
        sdf=(f'<sdf version="1.6"><model name="segmented_test_block"><static>true</static>'
             f'<link name="body"><collision name="collision">{geometry}</collision>'
             f'<visual name="visual">{geometry}</visual></link></model></sdf>')
        model=Pose(position=Point(center[0],center[1],1.3))
        model.orientation.w=1.
        assert spawn('segmented_test_block',sdf,'',model,'world').success
        start,_=decode(self.odom.pose.pose)
        name=self.new('obstacle_takeoff',takeoff=True,waypoint=(start[0],start[1],1.6))
        result=self.wait_phase(name,('DONE','FAILED'),40)
        assert result['phase']=='DONE',result
        cloud=rospy.wait_for_message('/grid_map/occupancy_inflate',PointCloud2,timeout=10)
        p,_=decode(self.odom.pose.pose)
        expected=(start[0]+.73,start[1],p[2])
        cells=[tuple(q) for q in point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True)
               if math.dist(q,expected)<.35]
        assert cells,'physical obstacle has no inflated occupied cells'
        blocked=min(cells,key=lambda q:math.dist(q,expected))
        name=self.new('occupied',waypoint=blocked)
        result=self.wait_phase(name,('DONE','FAILED'),20)
        assert result['phase']=='FAILED' and result['reason']=='planner_rejected',result
        assert result['segment_count']==0 and self.flight['context'] is None
        self.check('real_occupied_goal_rejected',blocked=blocked)
        name=self.new('bypass',waypoint=(start[0]+1.8,start[1],1.6))
        seen=[]
        deadline=time.monotonic()+65.
        while time.monotonic()<deadline:
            feedback=link(body,'world')
            assert feedback.success
            p=feedback.link_state.pose.position
            seen.append((p.x,p.y,p.z))
            if self.mission['mission_id']==name and self.mission['phase'] in ('DONE','FAILED'):
                break
            time.sleep(.08)
        assert self.mission['mission_id']==name and self.mission['phase']=='DONE',self.mission
        if self.mission.get('detour_active') is not None:
            staged = [d for _, kind, d in self.trace if kind=='mission' and d['mission_id']==name
                      and d['phase']=='TRACK' and d.get('detour_active')]
            assert len({d['goal_id'] for d in staged})>=3, staged
        surface=lambda p: math.hypot(max(0.,abs(p[0]-center[0])-size[0]/2),
                                     max(0.,abs(p[1]-center[1])-size[1]/2))
        clearance=min(surface(p) for p in seen)
        assert clearance>.24 and seen[-1][0]>center[0]+.45, (clearance,seen[-1])
        self.check('real_obstacle_bypassed',minimum_center_to_box_surface_m=clearance,
                   lateral_excursion_m=max(abs(p[1]-center[1]) for p in seen),world_samples=len(seen))
        self.record('world_obstacle_track',dict(center=center,size=size,samples=seen))
        self.stop_and_land()
        self.check('landed_disarmed')

    def delays(self):
        self.initial()
        for delay in (.1,3.,10.,12.8):
            name=self.new('delay',takeoff=delay==.1)
            p,yaw=decode(self.odom.pose.pose)
            self.scenarios[name]['target']=(p[0]+.8*math.cos(yaw),p[1]+.8*math.sin(yaw),.3)
            self.scenarios[name]['delay']=delay
            result=self.wait_phase(name,('DONE','FAILED'),26)
            expected='FAILED' if delay>12. else 'DONE'
            assert result['phase']==expected and result['segment_count']==0,result
            assert self.flight['mode']=='HOLD' and self.flight['context'] is None
            if expected=='FAILED':
                assert result['reason']=='detection_timeout'
                self.wait(lambda:any(label=='detect_result' and d['mission_id']==name
                    for _,label,d in self.trace),4,'reply past deadline')
                assert self.mission['phase']=='FAILED' and self.flight['context'] is None
            wait_begin=next(t for t,k,d in self.trace if k=='mission' and
                            d['mission_id']==name and d['phase']=='WAIT_VLM')
            commands=[d for t,k,d in self.trace if t>=wait_begin and k=='setpoint']
            assert commands and all(math.dist(c['position'],commands[0]['position'])<1e-6 for c in commands)
            self.check('detector_delay',delay_s=delay,outcome=expected)
        self.stop_and_land()
        self.check('landed_disarmed')

    def faults(self):
        def failed(name,reason,timeout=28):
            result=self.wait_phase(name,('DONE','FAILED'),timeout)
            assert result['phase']=='FAILED' and result['reason']==reason,result
            assert self.flight['mode']=='HOLD' and self.flight['context'] is None
            self.check(reason)
        for behavior,reason in (('missing','target_not_found'),('late','detection_timeout'),('jump','target_jump')):
            name=self.new(behavior,behavior)
            failed(name,reason)
            if behavior=='late':
                self.wait(lambda:any(label=='detect_result' and data['mission_id']==name
                    for _,label,data in self.trace),5,'late result delivered')
                assert self.mission['phase']=='FAILED' and self.flight['context'] is None
                self.check('late_detection_cannot_resume')

        failed(self.new('busy','busy'),'detector_busy')
        failed(self.new('network','network'),'api_unavailable')
        name=self.new('stale','stale')
        self.wait(lambda:any(label=='detect_result' and data['mission_id']==name
            for _,label,data in self.trace),8,'stale observation delivered')
        result=self.wait_phase(name,('DONE','FAILED'),8)
        assert result['reason']=='detection_timeout' and result['segment_count']==0,result
        assert self.flight['mode']=='HOLD' and self.flight['context'] is None
        self.check('stale_capture_never_plans')

        name=self.new('retired','late')
        self.wait_phase(name,('WAIT_VLM',),8)
        self.wait(lambda: self.scenarios[name]['calls']>0,3,'old observation sent')
        replacement=self.new('replacement','missing')
        failed(replacement,'target_not_found')
        self.wait(lambda:any(label=='detect_result' and data['mission_id']==name
            for _,label,data in self.trace),5,'retired result delivered')
        assert self.mission['mission_id']==replacement and self.flight['context'] is None
        self.check('replaced_task_cannot_resume')

        p,y=decode(self.odom.pose.pose)
        reachable=(p[0]+.7*math.cos(y),p[1]+.7*math.sin(y),1.6)
        self.plan_policy='no_route'
        failed(self.new('no_route',waypoint=reachable),'planner_rejected')
        self.plan_policy='outside_fence'
        name=self.new('bad_curve',waypoint=reachable)
        failed(name,'trajectory_outside_fence',20)
        self.check('out_of_fence_spline_never_executes')
        self.plan_policy='path'
        failed(self.new('fence',waypoint=(100.,0.,1.6)),'outside_fence')
        name=self.new('cancel',waypoint=reachable)
        self.wait_phase(name,('TRACK',),10)
        retired=self.last_spline
        self.send('CANCEL',mission_id=name)
        failed(name,'cancelled')
        retired.traj_id=999
        retired.start_time=rospy.Time.now()
        self.spline.publish(retired)
        time.sleep(.4)
        assert self.flight['context'] is None and self.flight['mode']=='HOLD'
        self.check('late_trajectory_cannot_resume')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mock',action='store_true')
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--obstacle',action='store_true')
    parser.add_argument('--delay-check',action='store_true')
    parser.add_argument('--target', default='画面中红色三脚支架中央的竖直红杆，仅框住中央红杆的红色像素，忽略支架其余部分和白色地面')
    parser.add_argument('--yaw',type=float)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    port=11315 if args.mock else 11311
    if (os.environ.get('ROS_MASTER_URI')!=f'http://127.0.0.1:{port}'
            or args.mock and (args.live or args.obstacle) or args.live and args.obstacle
            or args.delay_check and not args.mock):
        parser.error('unexpected ROS master for this run')
    rospy.init_node('segment_validation',anonymous=True)
    if not args.mock:
        rospy.wait_for_service('/gazebo/get_link_state',timeout=5)
        link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)('iris_depth_camera::iris::base_link','world')
        assert link.success
    trial=Trial(args)
    error=None
    try:
        if args.delay_check:
            trial.delays()
        elif args.obstacle:
            trial.obstacle()
        else:
            trial.normal()
    except Exception as exc:
        error=repr(exc)
        print('FAIL',error,flush=True)
        try:
            trial.stop_and_land()
        except Exception as stop_error:
            print('CLEANUP',repr(stop_error),flush=True)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'results.json').write_text(json.dumps(dict(passed=error is None,error=error,
        checks=trial.checks,final_armed=trial.state.armed if trial.state else None),indent=2)+'\n')
    (args.out/'trace.json').write_text(json.dumps(trial.trace)+'\n')
    if trial.mock:
        trial.mock.done.set()
        trial.mock.worker.join(timeout=2)
    if error:
        raise SystemExit(1)


if __name__=='__main__':
    main()
