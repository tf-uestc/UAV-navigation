#!/usr/bin/env python3
"""Coordinate segmented observations through the verified planner and flight executor."""
import json
import math
import threading
import time
import uuid

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from navigation_core.contracts import DetectionResult, RequestKey, vector3
from navigation_core.lifecycle import CallBudget, RequestRegistry
from navigation_core.targets import ArrivalTracker, TargetEstimator, observation_goal
from navigation_core.yaw import quaternion_yaw


TERMINAL = {'IDLE', 'DONE', 'FAILED', 'LANDED'}
PLAN_FAILURES = {'FAILED', 'REJECTED', 'STOP', 'MAP_NOT_READY', 'SESSION_CHANGED', 'CANCEL_TIMEOUT'}


class SegmentMission:
    def __init__(self):
        rospy.init_node('segment_mission')
        self.lock = threading.RLock()
        self.session = uuid.uuid4().hex
        self.seen = set()
        self.mission = None
        self.phase, self.reason = 'IDLE', ''
        self.deadline = self.started = 0.
        self.flight = self.planner = self.odom = None
        self.flight_session = None
        self.epoch = None
        self.seq = self.goal_id = self.stop_id = 0
        self.goal = self.final_goal = None
        self.target = None
        self.mode = None
        self.search = []
        self.search_index = 0
        self.after_stop = None
        self.settled_at = self.last_ros = None
        self.intended_yaw = None
        self.segment_count = 0
        self.recovery_count = 0
        self.registry = RequestRegistry()
        self.estimator = TargetEstimator(max_age=15.,max_jump=1.5)
        self.budget = CallBudget(max_calls=rospy.get_param('~max_detections',8),max_seconds=150.)
        self.arrival = ArrivalTracker(position_tolerance=.23,speed_tolerance=.15,dwell=.45)
        self.fence = rospy.get_param('/uav/segment_config/fence')
        self.height = rospy.get_param('~height',1.6)
        self.standoff = rospy.get_param('~standoff',2.4)
        self.stride = rospy.get_param('~segment_length',1.15)
        self.manual_stride = rospy.get_param('~manual_segment_length',self.stride)
        self.vlm_timeout = rospy.get_param('~detection_timeout',12.)
        self.max_segments = rospy.get_param('~max_segments',12)
        self.status_pub = rospy.Publisher('/uav/segment/status',String,queue_size=10,latch=True)
        self.flight_pub = rospy.Publisher('/uav/flight/command',String,queue_size=5)
        self.plan_pub = rospy.Publisher('/uav/planning/request',String,queue_size=5)
        self.detect_pub = rospy.Publisher('/uav/instruction',String,queue_size=5)
        rospy.Subscriber('/uav/segment/command',String,self.on_command,queue_size=10)
        rospy.Subscriber('/uav/flight/status',String,self.on_flight,queue_size=10)
        rospy.Subscriber('/uav/planning/readiness',String,self.on_ready,queue_size=2)
        rospy.Subscriber('/uav/planning/result',String,self.on_plan,queue_size=10)
        rospy.Subscriber('/uav/detection_result',String,self.on_detection,queue_size=10)
        rospy.Subscriber('/mavros/local_position/odom',Odometry,self.on_odom,queue_size=2)
        self.publish()

    @staticmethod
    def object(msg):
        try:
            data = json.loads(msg.data)
        except (TypeError,ValueError):
            return {}
        return data if isinstance(data,dict) else {}

    def publish(self):
        pending = self.registry.pending
        self.status_pub.publish(String(data=json.dumps(dict(
            mission_session_id=self.session,mission_id=self.mission,phase=self.phase,
            reason=self.reason,goal_id=self.goal_id,goal=self.goal,final_goal=self.final_goal,
            detection_request_id=pending.key.request_id if pending else None,
            intended_yaw=self.intended_yaw,segment_count=self.segment_count,
            capture_floor=self.settled_at,stop_id=self.stop_id),allow_nan=False)))

    def transition(self, phase, seconds=0., reason=''):
        self.phase,self.deadline,self.reason = phase,time.monotonic()+seconds,reason
        self.publish()

    def on_odom(self,msg):
        try:
            p,q,v,w = (msg.pose.pose.position,msg.pose.pose.orientation,
                       msg.twist.twist.linear,msg.twist.twist.angular)
            if msg.header.frame_id!='map' or msg.child_frame_id!='base_link':
                return
            sample = (vector3((p.x,p.y,p.z)), vector3((v.x,v.y,v.z)),
                      quaternion_yaw((q.x,q.y,q.z,q.w)), math.hypot(w.x,w.y,w.z),
                      msg.header.stamp.to_sec(),time.monotonic())
            if not math.isfinite(sample[3]):
                return
        except ValueError:
            return
        with self.lock:
            self.odom = sample

    def on_flight(self,msg):
        data = self.object(msg)
        if not isinstance(data.get('flight_session_id'),str) or type(data.get('seq')) is not int:
            return
        with self.lock:
            self.flight = (data,time.monotonic())
            self.seq = max(self.seq,data['seq'])

    def on_ready(self,msg):
        data = self.object(msg)
        if data.get('schema_version')==1:
            with self.lock:
                self.planner = (data,time.monotonic())

    def feedback(self):
        if self.odom is None or time.monotonic()-self.odom[-1]>.5:
            return None
        age = rospy.Time.now().to_sec()-self.odom[-2]
        return self.odom if -.1<=age<=.5 else None

    def flight_status(self):
        if self.flight and time.monotonic()-self.flight[1]<.7:
            return self.flight[0]
        return {}

    def planner_status(self):
        if self.planner and time.monotonic()-self.planner[1]<.7:
            data=self.planner[0]
            if data.get('ready') is True and all(isinstance(data.get(k),str) and data[k]
                    for k in ('gateway_session_id','planner_session_id')):
                return data
        return {}

    def command_flight(self,command,**fields):
        status = self.flight_status()
        if not status or status.get('flight_session_id')!=self.flight_session:
            return None
        self.seq += 1
        self.flight_pub.publish(String(data=json.dumps(dict(flight_session_id=self.flight_session,
            seq=self.seq,command=command,**fields),allow_nan=False)))
        return self.seq

    def inside(self,position):
        return all(self.fence[axis+'_min']<=value<=self.fence[axis+'_max']
                   for axis,value in zip('xyz',position))

    def on_command(self,msg):
        d = self.object(msg)
        if d.get('mission_session_id')!=self.session:
            return
        with self.lock:
            cmd=d.get('command')
            flight=self.flight_status()
            if cmd=='LAND' and flight.get('owns_flight'):
                self.registry.cancel()
                if self.command_flight('LAND') is not None:
                    self.transition('LANDING',40.)
                return
            if cmd=='CANCEL' and self.phase not in TERMINAL and self.phase!='LANDING':
                if d.get('mission_id')!=self.mission:
                    return
                if self.phase=='TAKEOFF':
                    self.registry.cancel()
                    self.command_flight('LAND')
                    self.transition('LANDING',40.,'cancelled')
                else:
                    self.brake('FAILED','cancelled')
                return
            if cmd!='START' or self.phase=='LANDING' or not flight.get('ready') or not self.feedback():
                return
            try:
                mission=d['mission_id']
                RequestKey(mission,1)
                if mission in self.seen:
                    return
                manual=vector3(d['waypoint']) if 'waypoint' in d else None
                target=d.get('target')
                if manual is None and (not isinstance(target,str) or not target.strip()):
                    return
                # Waypoints describe additional observations after a genuine no-target response.
                search=[vector3(p) for p in d.get('search_waypoints',[])]
                if len(search)>6:
                    return
            except (TypeError,ValueError,KeyError):
                return
            if self.phase=='TAKEOFF' or (flight.get('owns_flight') and flight.get('mode') not in ('HOLD','TRACK')):
                return
            if not flight.get('owns_flight') and d.get('takeoff') is not True:
                return
            self.seen.add(mission)
            self.flight_session=flight['flight_session_id']
            self.mission,self.target,self.mode=mission,target,'manual' if manual is not None else 'visual'
            self.goal,self.final_goal=None,manual
            self.goal_id,self.segment_count,self.stop_id=0,0,0
            self.recovery_count=0
            self.search,self.search_index=search,0
            self.registry.begin(mission)
            self.estimator.reset()
            self.budget.begin(time.monotonic())
            self.arrival=ArrivalTracker(position_tolerance=.23,speed_tolerance=.15,dwell=.45)
            self.started=time.monotonic()
            self.intended_yaw=self.feedback()[2]
            if (manual is not None and not self.inside(manual)) or any(not self.inside(p) for p in search):
                if flight.get('owns_flight'):
                    self.brake('FAILED','outside_fence')
                else:
                    self.transition('FAILED',reason='outside_fence')
            elif flight.get('owns_flight'):
                self.brake('PLAN' if self.mode=='manual' else 'DETECT')
            else:
                self.command_flight('TAKEOFF',height=self.height)
                self.transition('TAKEOFF',34.)

    def brake(self, after, reason=''):
        self.registry.cancel()
        self.after_stop=(after,reason)
        self.settled_at=None
        self.stop_id=self.command_flight('HOVER')
        if self.stop_id is None:
            self.transition('FAILED',reason=reason or 'flight_session_unavailable')
        else:
            self.transition('BRAKE',8.,reason)

    def request_detection(self,now):
        if not self.budget.reserve(now):
            self.transition('FAILED',reason='detection_budget_exhausted')
            return
        self.registry.begin(self.mission)
        self.recovery_count=0
        key=self.registry.issue(self.target,now,self.vlm_timeout)
        self.transition('WAIT_VLM',self.vlm_timeout)
        self.detect_pub.publish(String(data=json.dumps(dict(target={'text':self.target},
            _mission_mgr_trigger=True,mission_id=self.mission,request_id=key.request_id))))

    def on_detection(self,msg):
        try:
            result=DetectionResult.from_json(msg.data)
        except (TypeError,ValueError,KeyError):
            return
        with self.lock:
            if self.phase!='WAIT_VLM':
                return
            now=time.monotonic()
            stamp=rospy.Time.now().to_sec()
            if result.status=='SUCCEEDED' and result.capture_stamp<self.settled_at:
                return
            if not self.registry.resolve(result,now,ros_now=stamp,max_age=15.):
                return
            if result.status!='SUCCEEDED':
                if (result.reason=='target_not_found' and self.estimator.point is None
                        and self.search_index<len(self.search)):
                    self.final_goal=self.search[self.search_index]
                    self.search_index+=1
                    self.plan_segment('search')
                else:
                    self.transition('FAILED',reason=result.reason or result.status.lower())
                return
            if not self.estimator.observe(result.point,result.capture_stamp,stamp):
                self.transition('FAILED',reason='target_'+self.estimator.reason)
                return
            fb=self.feedback()
            if not fb:
                self.transition('FAILED',reason='odometry_stale')
                return
            try:
                actual=fb[0]
                final=observation_goal(actual,result.point,self.height,self.standoff,self.fence)
                if math.dist(actual[:2],result.point[:2])<=self.standoff+.23:
                    self.final_goal=actual
                    self.transition('DONE',reason='observation_standoff_reached')
                    return
                self.final_goal=final.position
            except (KeyError,ValueError):
                self.transition('FAILED',reason='observation_outside_fence')
                return
            self.plan_segment('approach')

    def plan_segment(self,kind):
        fb=self.feedback()
        if not fb:
            self.transition('FAILED',reason='odometry_stale')
            return
        start=fb[0]
        distance=math.dist(start,self.final_goal)
        if distance<=.23:
            if kind=='search':
                self.request_detection(time.monotonic())
            else:
                self.transition('DONE',reason='destination_reached')
            return
        if self.segment_count>=self.max_segments:
            self.transition('FAILED',reason='segment_budget_exhausted')
            return
        stride=self.manual_stride if kind=='manual' else self.stride
        ratio=min(1.,stride/distance)
        self.goal=tuple(a+(b-a)*ratio for a,b in zip(start,self.final_goal))
        if not self.inside(self.goal):
            self.transition('FAILED',reason='segment_outside_fence')
            return
        self.kind=kind
        self.goal_id+=1
        self.epoch=None
        self.transition('AUTHORIZE',5.)

    def on_plan(self,msg):
        d=self.object(msg)
        with self.lock:
            if (self.phase not in ('WAIT_PLAN','TRACK') or not self.epoch
                    or (d.get('mission_id'),d.get('goal_id'))!=(self.mission,self.goal_id)
                    or (d.get('gateway_session_id'),d.get('planner_session_id'))!=self.epoch):
                return
            if d.get('status') in PLAN_FAILURES:
                self.brake('FAILED','planner_'+d['status'].lower())

    def tick(self,now):
        if self.phase in TERMINAL:
            return
        flight=self.flight_status()
        stamp=rospy.Time.now().to_sec()
        if self.last_ros is not None and stamp<self.last_ros:
            self.brake('FAILED','ros_clock_reset')
            self.last_ros=stamp
            return
        self.last_ros=stamp
        if flight and flight['flight_session_id']!=self.flight_session:
            self.registry.cancel()
            self.transition('FAILED',reason='flight_session_changed')
            return
        if self.phase=='LANDING':
            if flight and not flight.get('owns_flight') and flight.get('mode')=='COMPLETE':
                self.transition('LANDED',reason=self.reason)
            elif now>=self.deadline:
                self.transition('FAILED',reason='landing_feedback_timeout')
            return
        if self.phase=='TAKEOFF':
            if flight.get('mode')=='HOLD' and flight.get('owns_flight'):
                self.brake('PLAN' if self.mode=='manual' else 'DETECT')
            elif flight.get('mode')=='FAILED' or now>=self.deadline:
                self.command_flight('LAND')
                self.transition('FAILED',reason='takeoff_failed')
            return
        if not flight or not flight.get('owns_flight') or flight.get('mode') in ('FAILED','LANDING','COMPLETE'):
            self.registry.cancel()
            self.transition('FAILED',reason='flight_feedback_unavailable')
            return
        if now-self.started>150. and self.phase not in ('BRAKE','SETTLE'):
            self.brake('FAILED','mission_deadline')
            return
        if self.phase in ('BRAKE','SETTLE'):
            fb=self.feedback()
            if (self.phase=='BRAKE' and flight.get('stop_id')==self.stop_id
                    and flight.get('mode')=='HOLD' and flight.get('context') is None):
                self.transition('SETTLE',7.,self.reason)
            still = fb and math.sqrt(sum(v*v for v in fb[1]))<=.15 and fb[3]<=.12
            if self.phase=='SETTLE' and still and flight.get('stop_id')==self.stop_id and flight.get('stop_confirmed'):
                if self.settled_at is None:
                    self.settled_at=stamp
                    self.intended_yaw=fb[2]
                if stamp>=self.settled_at+.18:
                    after,reason=self.after_stop
                    if after=='DETECT':
                        self.request_detection(now)
                    elif after=='PLAN':
                        self.plan_segment('manual' if self.mode=='manual' else self.kind)
                    elif after=='REPLAN':
                        self.plan_segment(self.kind)
                    else:
                        self.transition(after,reason=reason)
                    return
            else:
                self.settled_at=None
            if now>=self.deadline:
                self.command_flight('LAND')
                self.transition('FAILED',reason='stop_confirmation_timeout')
        elif self.phase=='WAIT_VLM':
            if now>=self.deadline:
                self.registry.cancel()
                self.transition('FAILED',reason='detection_timeout')
        elif self.phase=='AUTHORIZE':
            ready=self.planner_status()
            if not self.epoch and ready:
                self.epoch=ready['gateway_session_id'],ready['planner_session_id']
                self.command_flight('BEGIN',mission_id=self.mission,goal_id=self.goal_id,
                    gateway_session_id=self.epoch[0],planner_session_id=self.epoch[1])
            if self.epoch and flight.get('context')==[self.mission,self.goal_id,self.epoch[1]]:
                self.transition('WAIT_PLAN',9.)
                self.plan_pub.publish(String(data=json.dumps(dict(schema_version=1,frame_id='map',
                    mission_id=self.mission,goal_id=self.goal_id,gateway_session_id=self.epoch[0],
                    goal=self.goal))))
            elif now>=self.deadline:
                self.brake('FAILED','planning_authorization_timeout')
        elif self.phase=='WAIT_PLAN':
            if flight.get('mode')=='TRACK' and flight.get('context')==[self.mission,self.goal_id,self.epoch[1]]:
                self.segment_count+=1
                self.transition('TRACK',20.)
            elif flight.get('reason')=='trajectory_outside_fence':
                self.brake('FAILED','trajectory_outside_fence')
            elif now>=self.deadline:
                self.brake('FAILED','planning_timeout')
        elif self.phase=='TRACK':
            fb=self.feedback()
            if not fb or flight.get('context')!=[self.mission,self.goal_id,self.epoch[1]]:
                if fb and flight.get('reason')=='discontinuous_trajectory' and self.recovery_count<1:
                    self.recovery_count+=1
                    self.brake('REPLAN','replan_from_settled_pose')
                else:
                    self.brake('FAILED','trajectory_lost')
            elif self.arrival.update(fb[0],fb[1],self.goal,now):
                if self.mode=='manual' or self.kind=='search' and math.dist(fb[0],self.final_goal)>.23:
                    self.brake('PLAN')
                else:
                    self.brake('DETECT')
            elif now>=self.deadline:
                self.brake('FAILED','arrival_timeout')

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.tick(time.monotonic())
            time.sleep(.04)


if __name__=='__main__':
    SegmentMission().run()
