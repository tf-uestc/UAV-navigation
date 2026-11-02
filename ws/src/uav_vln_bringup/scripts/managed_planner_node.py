#!/usr/bin/env python3
"""Identified EGO planning only. Publishes no MAVROS or goal_pose commands."""
import json
import threading
import time
import uuid

import rospy
from ego_planner.msg import Bspline, GoalCommand, GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
from navigation_core.bspline import BSpline
from navigation_core.contracts import RequestKey, vector3


class ManagedPlanner:
    def __init__(self):
        rospy.init_node('managed_planner')
        self.lock=threading.RLock()
        self.gateway=uuid.uuid4().hex
        self.session=None
        self.ready=False
        self.status_wall=0.
        self.active=None
        self.cancelling={}
        self.highest={}
        self.cancelled_through={}
        self.retired=set()
        self.retired_sessions=set()
        self.current_mission=None
        self.timeout=rospy.get_param('~plan_timeout',8.)
        self.command_pub=rospy.Publisher('/uav/ego/command',GoalCommand,queue_size=10)
        self.result_pub=rospy.Publisher('/uav/planning/result',String,queue_size=30)
        self.readiness_pub=rospy.Publisher('/uav/planning/readiness',String,queue_size=2,latch=True)
        self.spline_pub=rospy.Publisher('/uav/planning/trajectory',Bspline,queue_size=10)
        self.preview_pub=rospy.Publisher('/uav/trajectory_preview',Path,queue_size=1,latch=True)
        rospy.Subscriber('/uav/ego/status',GoalStatus,self.on_status,queue_size=30)
        rospy.Subscriber('/planning/bspline',Bspline,self.on_spline,queue_size=10)
        rospy.Subscriber('/uav/planning/request',String,self.on_request,queue_size=10)
        rospy.Subscriber('/uav/cancel_planning',String,self.on_cancel,queue_size=10)

    def emit(self,key,status,reason='',**extra):
        data=dict(schema_version=1,mission_id=key[0],goal_id=key[1],status=status,
                  reason=reason,gateway_session_id=self.gateway,planner_session_id=self.session)
        data.update(extra)
        self.result_pub.publish(String(data=json.dumps(data,allow_nan=False)))

    def available(self):
        return bool(self.ready and self.session and time.monotonic()-self.status_wall<.6)

    def send(self,key,command,goal=None):
        m=GoalCommand(header=Header(stamp=rospy.Time.now(),frame_id='map'),
                      planner_session_id=self.session or '',mission_id=key[0],goal_id=key[1],command=command)
        if goal is not None: m.goal=Point(*goal)
        self.command_pub.publish(m)

    def revoke(self,status,reason):
        if self.active:
            key=self.active['key']
            self.active=None
            self.cancelling[(self.session,key)]=time.monotonic()+1.
            self.emit(key,'CANCELLING' if status=='CANCELLED' else status,reason)
            self.send(key,'CANCEL')

    def on_cancel(self,msg):
        with self.lock:
            try:
                data=json.loads(msg.data)
            except (ValueError,TypeError):
                data=None
            if isinstance(data,dict):
                if data.get('schema_version')!=1 or data.get('gateway_session_id')!=self.gateway:return
                mission=data.get('mission_id');through=data.get('through_goal_id')
                if not isinstance(mission,str) or not mission or type(through) is not int or through<0:return
                self.cancelled_through[mission]=max(through,self.cancelled_through.get(mission,0))
                if not self.active or self.active['key'][0]!=mission or self.active['key'][1]>through:return
            self.revoke('CANCELLED','requested')

    def on_request(self,msg):
        try:
            d=json.loads(msg.data)
            if type(d.get('schema_version')) is not int or d['schema_version']!=1: return
            RequestKey(d['mission_id'],d['goal_id'])
            goal=vector3(d['goal'])
            if d.get('frame_id')!='map': return
            key=(d['mission_id'],d['goal_id'])
        except (ValueError,TypeError,KeyError,AttributeError):
            return
        with self.lock:
            if d.get('gateway_session_id')!=self.gateway:
                self.emit(key,'REJECTED','gateway_session_mismatch'); return
            if self.active and self.active['key']==key:
                if goal!=self.active['goal']: self.emit(key,'REJECTED','goal_identity_reused')
                else: self.emit(key,'ACCEPTED' if self.active['accepted'] else 'PLANNING','duplicate_request')
                return
            if key[0] in self.retired or key[1]<=max(self.highest.get(key[0],0),self.cancelled_through.get(key[0],0)):
                self.emit(key,'REJECTED','retired_goal'); return
            if not self.available():
                self.emit(key,'MAP_NOT_READY','missing_fresh_ego_map'); return
            self.revoke('CANCELLED','superseded')
            if self.current_mission and self.current_mission!=key[0]: self.retired.add(self.current_mission)
            self.current_mission=key[0]
            self.highest[key[0]]=key[1]
            self.active=dict(key=key,goal=goal,accepted=False,pending=None,seen=set(),
                             deadline=time.monotonic()+self.timeout)
            self.send(key,'GOAL',goal)

    def on_status(self,msg):
        with self.lock:
            if msg.planner_session_id in self.retired_sessions: return
            if self.session and msg.planner_session_id!=self.session:
                self.retired_sessions.add(self.session)
                # Never automatically replay an active goal into a restarted EGO.
                if self.active:
                    key=self.active['key']; self.active=None
                    self.emit(key,'SESSION_CHANGED','ego_restarted')
            self.session=msg.planner_session_id
            self.ready=msg.map_ready
            self.status_wall=time.monotonic()
            cancelled=(msg.planner_session_id,(msg.mission_id,msg.goal_id))
            if msg.status=='CANCELLED' and cancelled in self.cancelling:
                del self.cancelling[cancelled]
                self.emit(cancelled[1],'CANCELLED','ego_acknowledged')
            if not self.active or (msg.mission_id,msg.goal_id)!=self.active['key']: return
            key=self.active['key']
            if msg.status=='ACCEPTED':
                self.active['accepted']=True
                self.emit(key,'ACCEPTED')
                pending=self.active['pending']; self.active['pending']=None
                if pending is not None: self.forward(pending)
            elif msg.status in ('FAILED','CANCELLED','REJECTED','COMPLETED'):
                self.emit(key,msg.status,msg.reason)
                self.active=None
            elif msg.status=='NOT_READY':
                self.revoke('MAP_NOT_READY',msg.reason or 'map_or_odometry_stale')

    def on_spline(self,msg):
        with self.lock:
            if not self.active or msg.planner_session_id!=self.session: return
            if (msg.mission_id,msg.goal_id)!=self.active['key']: return
            if not self.active['accepted']:
                self.active['pending']=msg
                return
            self.forward(msg)

    def forward(self,msg):
        if not self.active: return
        identity=(msg.traj_id,msg.start_time.to_nsec())
        if identity in self.active['seen']: return
        try:
            spline=BSpline([(p.x,p.y,p.z) for p in msg.pos_pts],msg.knots,msg.order)
            samples=spline.samples()
            path=Path(header=Header(stamp=msg.start_time,frame_id='map'))
            for t,p,_v,_a in samples:
                pose=PoseStamped(header=Header(stamp=msg.start_time+rospy.Duration(t),frame_id='map'))
                pose.pose.position=Point(*p); pose.pose.orientation.w=1.
                path.poses.append(pose)
        except (ValueError,TypeError,IndexError):
            self.revoke('FAILED','invalid_spline'); return
        self.active['seen'].add(identity)
        self.active['deadline']=None
        self.spline_pub.publish(msg)  # Preserve every original knot/control point/start time.
        self.preview_pub.publish(path)
        self.emit(self.active['key'],'STOP' if msg.emergency_stop else 'OK',traj_id=msg.traj_id)

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                for pending,deadline in list(self.cancelling.items()):
                    if time.monotonic()>=deadline:
                        del self.cancelling[pending]
                        self.emit(pending[1],'CANCEL_TIMEOUT','ego_cancel_ack_missing')
                if self.active and self.active['deadline'] is not None and time.monotonic()>=self.active['deadline']:
                    self.revoke('FAILED','planning_deadline')
                if self.active and not self.available():
                    self.revoke('MAP_NOT_READY','ego_heartbeat_stale')
                self.readiness_pub.publish(String(data=json.dumps(dict(schema_version=1,
                    gateway_session_id=self.gateway,planner_session_id=self.session,ready=self.available()))))
            time.sleep(.05)


if __name__=='__main__':
    ManagedPlanner().run()
