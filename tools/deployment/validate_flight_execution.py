#!/usr/bin/env python3
"""ROS integration checks with a mock FCU or an explicitly selected PX4 SITL."""
import argparse
import copy
import json
import math
import os
import subprocess
import threading
import time
import uuid
from pathlib import Path

import rosgraph
import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TwistStamped, Vector3
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from tf.transformations import quaternion_matrix
from navigation_core.yaw import angle_delta, quaternion, quaternion_yaw


def decode(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z), quaternion_yaw((q.x, q.y, q.z, q.w))


class FakeFcu:
    def __init__(self):
        self.position = (0., 0., 0.)
        self.yaw = math.pi-.02
        self.target = None
        self.armed = False
        self.mode = 'POSCTL'
        self.allow_arm = True
        self.connected = True
        self.publish_odom = True
        self.fixed_position = False
        self.reject_mode = False
        self.arm_calls = 0
        self.state = rospy.Publisher('/mavros/state', State, queue_size=1)
        self.landed = rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1)
        self.odom = rospy.Publisher('/mavros/local_position/odom', Odometry, queue_size=1)
        rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, self.setpoint, queue_size=1)
        rospy.Service('/mavros/cmd/arming', CommandBool, self.arm)
        rospy.Service('/mavros/set_mode', SetMode, self.set_mode)
        self.done = threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def setpoint(self, msg):
        self.target = decode(msg.pose)

    def arm(self, request):
        self.arm_calls += int(request.value)
        if request.value and not self.allow_arm:
            return CommandBoolResponse(success=False)
        self.armed = request.value
        return CommandBoolResponse(success=True)

    def set_mode(self, request):
        if self.reject_mode and request.custom_mode == 'OFFBOARD':
            return SetModeResponse(mode_sent=False)
        self.mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def run(self):
        while not self.done.is_set() and not rospy.is_shutdown():
            position = self.position
            if self.mode == 'AUTO.LAND':
                self.position = (*position[:2], max(0., position[2]-.035))
            elif self.armed and self.mode == 'OFFBOARD' and self.target and not self.fixed_position:
                self.position = tuple(a+.14*(b-a) for a,b in zip(position,self.target[0]))
                self.yaw = self.target[1]
            stamp = rospy.Time.now()
            self.state.publish(State(header=Header(stamp=stamp), connected=self.connected,
                                     armed=self.armed, mode=self.mode))
            self.landed.publish(ExtendedState(header=Header(stamp=stamp),
                                               landed_state=1 if self.position[2] < .04 else 2))
            if self.publish_odom:
                msg = Odometry(header=Header(stamp=stamp, frame_id='map'), child_frame_id='base_link')
                msg.pose.pose.position = Point(*self.position)
                msg.pose.pose.orientation = Quaternion(*quaternion(self.yaw))
                msg.twist.twist.linear = Vector3(*((b-a)/.04 for a,b in zip(position,self.position)))
                self.odom.publish(msg)
            time.sleep(.04)


class SitlFaultInputs:
    """Inject faults at the executor's inputs while recording the native FCU."""
    def __init__(self):
        self.refuse_arm = False
        self.mask_height = False
        self.stale_stamp = False
        self.drop_link = False
        self.state = rospy.Publisher('/uav/flight_test/state', State, queue_size=1)
        self.odom = rospy.Publisher('/uav/flight_test/odom', Odometry, queue_size=1)
        self.inputs = [rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1),
                       rospy.Subscriber('/mavros/local_position/odom', Odometry, self.on_odom, queue_size=1)]
        rospy.Service('/uav/flight_test/arming', CommandBool, self.on_arm)

    def on_state(self, msg):
        if self.drop_link:
            msg = copy.deepcopy(msg)
            msg.connected = False
        self.state.publish(msg)

    def on_odom(self, msg):
        if self.mask_height or self.stale_stamp:
            msg = copy.deepcopy(msg)
            if self.mask_height:
                msg.pose.pose.position.z = min(msg.pose.pose.position.z,.25)
            if self.stale_stamp:
                msg.header.stamp -= rospy.Duration(2.)
        self.odom.publish(msg)

    def on_arm(self, request):
        if self.refuse_arm and request.value:
            return CommandBoolResponse(success=False)
        return rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)(request.value)


class Trial:
    def __init__(self, args):
        self.args = args
        self.status = self.state = self.landed = self.odom = self.ready = None
        self.points = []
        self.poses = []
        self.velocity_pairs = []
        self.world_velocity = None
        self.targets = []
        self.planner_results = []
        self.mode_history = []
        self.flight_history = []
        self.checks = []
        self.seq = 0
        self.last_mode = self.last_flight = None
        self.fake = FakeFcu() if args.mock else None
        self.fault_inputs = None
        self.replacement = None
        self.plan_pub = rospy.Publisher('/uav/planning/trajectory', Bspline, queue_size=10)
        self.readiness = rospy.Publisher('/uav/planning/readiness', String, queue_size=1)
        self.request = rospy.Publisher('/uav/planning/request', String, queue_size=5)
        self.cancel = rospy.Publisher('/uav/cancel_planning', String, queue_size=5)
        self.command = rospy.Publisher('/uav/flight/command', String, queue_size=5)
        self.mission = 'stage9-'+uuid.uuid4().hex
        self.goal = 0
        self.emit_ready = not args.real_ego
        self.gateway, self.epoch = 'stage9-gateway', 'stage9-ego'
        rospy.Subscriber('/uav/flight/status', String, self.status_cb, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=10)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self.save, 'landed', queue_size=1)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_cb, queue_size=5)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.save,
                         'world_velocity', queue_size=1)
        rospy.Subscriber('/mavros/setpoint_raw/target_local', PositionTarget, self.target_cb, queue_size=5)
        rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, self.point_cb, queue_size=20)
        rospy.Subscriber('/uav/planning/readiness', String, self.ready_cb, queue_size=2)
        rospy.Subscriber('/uav/planning/result', String,
                         lambda msg: self.planner_results.append(json.loads(msg.data)), queue_size=20)
        if not args.real_ego:
            threading.Thread(target=self.heartbeat, daemon=True).start()

    def heartbeat(self):
        while not rospy.is_shutdown():
            if self.emit_ready:
                try:
                    self.readiness.publish(String(data=json.dumps(dict(schema_version=1, ready=True,
                        gateway_session_id=self.gateway, planner_session_id=self.epoch))))
                except rospy.ROSException:
                    if rospy.is_shutdown():
                        return
                    raise
            time.sleep(.05)

    def ready_cb(self, msg):
        self.ready = json.loads(msg.data)

    def save(self, msg, field):
        setattr(self, field, msg)

    def status_cb(self, msg):
        self.status = json.loads(msg.data)
        key = (self.status['mode'], self.status['reason'])
        if key != self.last_flight:
            self.flight_history.append((time.monotonic(), key))
            self.last_flight = key

    def state_cb(self, msg):
        self.state = msg
        key = (msg.armed, msg.mode, msg.connected)
        if key != self.last_mode:
            self.mode_history.append((time.monotonic(), key))
            self.last_mode = key

    def odom_cb(self, msg):
        self.odom = msg
        p,yaw = decode(msg.pose.pose)
        v = msg.twist.twist.linear
        self.poses.append((time.monotonic(), p, yaw, math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)))
        world = self.world_velocity
        if world and abs((msg.header.stamp-world.header.stamp).to_sec()) < .025:
            q = msg.pose.pose.orientation
            predicted = quaternion_matrix((q.x,q.y,q.z,q.w))[:3,:3].dot([v.x,v.y,v.z])
            actual = world.twist.linear
            self.velocity_pairs.append((math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                                        math.dist(predicted,(actual.x,actual.y,actual.z))))

    def target_cb(self, msg):
        self.targets.append((time.monotonic(), msg.header.stamp.to_sec(),
                             msg.coordinate_frame, msg.type_mask, msg.yaw))

    def point_cb(self, msg):
        p,yaw = decode(msg.pose)
        self.points.append((time.monotonic(), msg.header.stamp.to_sec(), msg.header.frame_id, p,yaw))

    def wait(self, fn, seconds, label):
        end = time.monotonic()+seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            if fn():
                return
            time.sleep(.025)
        raise AssertionError(label+': '+str(self.status)+' / '+str(self.last_mode))

    def check(self, name, **details):
        self.checks.append(dict(name=name, **details))
        print('PASS',name,details,flush=True)

    def send(self, command, **payload):
        self.seq += 1
        msg = dict(flight_session_id=self.status['flight_session_id'], seq=self.seq,
                   command=command, **payload)
        self.command.publish(String(data=json.dumps(msg)))
        return self.seq

    def takeoff(self):
        self.wait(lambda: self.status['ready'] and not self.status['owns_flight']
                  and not self.state.armed and self.landed.landed_state==1, 8, 'ready for new takeoff')
        self.send('TAKEOFF', height=1.6)
        self.wait(lambda: self.status['mode']=='HOLD' and self.state.armed
                  and self.state.mode=='OFFBOARD' and self.landed.landed_state==2, 32, 'takeoff')

    def begin(self):
        if self.args.real_ego:
            self.wait(lambda: self.ready and self.ready['ready'], 10, 'EGO readiness')
            self.gateway = self.ready['gateway_session_id']
            self.epoch = self.ready['planner_session_id']
        self.goal += 1
        self.send('BEGIN', mission_id=self.mission, goal_id=self.goal,
                  gateway_session_id=self.gateway, planner_session_id=self.epoch)
        self.wait(lambda: self.status['context']==[self.mission,self.goal,self.epoch], 2, 'authorize context')

    def plan(self, start, end, trajectory_id=1, duration=4.):
        msg = Bspline(mission_id=self.mission, goal_id=self.goal, planner_session_id=self.epoch,
                      traj_id=trajectory_id, order=3, start_time=rospy.Time.now()+rospy.Duration(.12),
                      knots=[0.]*4+[duration]*4,
                      pos_pts=[Point(*p) for p in [start, start, end, end]])
        self.plan_pub.publish(msg)
        return msg

    def stop(self):
        seq = self.send('HOVER')
        self.wait(lambda: self.status['stop_id']==seq and self.status['context'] is None, 2, 'stop revoked')
        self.wait(lambda: self.status['stop_id']==seq and self.status['stop_confirmed'], 12, 'measured stop')
        return seq

    def land(self):
        self.send('LAND')
        self.wait(lambda: not self.state.armed and self.landed.landed_state==1
                  and not self.status['owns_flight'], 35, 'land and disarm')

    def run(self):
        self.wait(lambda: self.status and self.status['ready'] and self.odom and self.state and self.landed
                  and self.command.get_num_connections(), 35, 'initial status')
        assert not self.state.armed and self.landed.landed_state==1
        owners = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        for topic in ('/uav/goal_pose','/mavros/setpoint_position/local'):
            assert owners[topic]==['/flight_executor'], owners.get(topic)
        self.check('single_publisher',topics=['/uav/goal_pose','/mavros/setpoint_position/local'])
        if self.args.yaw is not None:
            since = None
            expected = math.radians(self.args.yaw)

            def heading_ready():
                nonlocal since
                _, yaw = decode(self.odom.pose.pose)
                if abs(math.degrees(angle_delta(yaw, expected))) >= 12.:
                    since = None
                    return False
                if since is None:
                    since = time.monotonic()
                return time.monotonic()-since >= 1.

            self.wait(heading_ready,30,'settled initial yaw')
        p,yaw = decode(self.odom.pose.pose)
        yaw_deg = math.degrees(yaw)
        self.takeoff()
        self.check('takeoff_actual_feedback',initial_yaw_deg=yaw_deg,
                   height=self.odom.pose.pose.position.z, mode=self.state.mode)
        self.begin()
        start,_ = decode(self.odom.pose.pose)
        end = (start[0]+.9,start[1],start[2])
        old = None
        t0 = time.monotonic()
        if self.args.real_ego:
            self.request.publish(String(data=json.dumps(dict(schema_version=1, frame_id='map',
                mission_id=self.mission,goal_id=self.goal,gateway_session_id=self.gateway,goal=end))))
        else:
            old = self.plan(start,end)
        self.wait(lambda: self.status['mode']=='TRACK' and self.status['trajectory_id'] is not None,
                  10, 'sample trajectory')
        self.wait(lambda: self.odom.pose.pose.position.x-start[0] > .35, 10, 'actual forward move')
        sent = [p for p in self.points if p[0]>=t0]
        assert any(.03 < item[3][0]-start[0] < .3 for item in sent), 'nearby trajectory point was suppressed'
        self.check('trajectory_moves_without_standoff', measured_dx=self.odom.pose.pose.position.x-start[0])
        if self.args.real_ego:
            self.cancel.publish(String(data=json.dumps(dict(schema_version=1,
                mission_id=self.mission,through_goal_id=self.goal,gateway_session_id=self.gateway))))
            self.wait(lambda: self.status['context'] is None and self.status['stop_confirmed'],
                      12,'planner cancellation and measured stop')
            self.wait(lambda: any(r['mission_id']==self.mission and r['goal_id']==self.goal
                      and r['status']=='CANCELLED' and r['reason']=='ego_acknowledged'
                      for r in self.planner_results),2,'EGO cancellation acknowledgment')
            self.check('native_ego_cancellation_acknowledged')
        else:
            self.stop()
        after = time.monotonic()
        if old:
            old.traj_id = 100
            old.start_time = rospy.Time.now()
            self.plan_pub.publish(old)
            self.send('BEGIN',mission_id=self.mission,goal_id=self.goal,
                      gateway_session_id=self.gateway,planner_session_id=self.epoch)
        time.sleep(.35)
        assert self.status['context'] is None and self.status['mode']=='HOLD',self.status
        self.check('stop_confirmed_and_retired_context',reason=self.status['reason'])

        end_hold = after+self.args.hold
        while time.monotonic() < end_hold:
            assert self.state.armed and self.state.mode=='OFFBOARD',self.last_mode
            time.sleep(.05)
        points = [p for p in self.points if after <= p[0] <= end_hold]
        poses = [p for p in self.poses if after <= p[0] <= end_hold]
        anchor = points[0][3]
        hold_yaw = points[0][4]
        max_drift = max(math.dist(p[1],anchor) for p in poses)
        max_yaw = max(abs(math.degrees(angle_delta(p[2],hold_yaw))) for p in poses)
        max_requested_yaw = max(abs(math.degrees(angle_delta(p[4],yaw))) for p in points)
        max_gap = max(b[0]-a[0] for a,b in zip(points,points[1:]))
        assert len(points) >= 18*self.args.hold and max_gap <= .2,(len(points),max_gap)
        assert all(p[2]=='map' and math.dist(p[3],anchor)<1e-6 for p in points)
        assert max_drift <= .3 and max_yaw <= 10. and max_requested_yaw <= 10.,(
            max_drift,max_yaw,max_requested_yaw)
        self.check('fixed_hold_and_heartbeat',seconds=self.args.hold, messages=len(points),
                   max_gap_s=max_gap,max_position_error_m=max_drift,max_yaw_error_deg=max_yaw,
                   max_command_yaw_from_initial_deg=max_requested_yaw)
        self.land()
        self.check('landing_actual_feedback',mode=self.state.mode)
        if self.args.sitl:
            moving = [e for speed,e in self.velocity_pairs if speed > .15]
            assert len(moving) >= 10 and max(moving) < .2, moving
            self.check('body_to_world_velocity_matches_mavros', moving_samples=len(moving),
                       max_velocity_error_mps=max(moving))
        self.send('TAKEOFF',height=1.6)
        self.wait(lambda: self.status['mode']=='PRESTREAM',2,'second prestream')
        self.land()
        time.sleep(1.)
        assert not self.state.armed and self.landed.landed_state==1
        self.check('land_preempts_prestream')
        if self.args.real_ego:
            self.send('TAKEOFF',height=1.6)
            self.wait(lambda: self.status['mode']=='CLIMB' and self.landed.landed_state==2
                      and self.odom.pose.pose.position.z > .25,12,'airborne climb for cancellation')
            self.land()
            self.check('land_preempts_airborne_climb')

        if self.fake:
            self.faults()

    def faults(self):
        fake = self.fake
        other = rospy.Publisher('/uav/goal_pose',PoseStamped,queue_size=1)
        self.wait(lambda: not self.status['owner'],2,'publisher conflict')
        count = fake.arm_calls
        self.send('TAKEOFF')
        time.sleep(2.2)
        assert fake.arm_calls==count and not self.state.armed
        other.unregister()
        self.wait(lambda:self.status['ready'],3,'publisher conflict cleared')
        self.check('conflicting_publisher_blocks_arming')
        fake.allow_arm = False
        self.send('TAKEOFF')
        self.wait(lambda: self.status['mode']=='WAIT_ARM',5,'arm denied')
        self.wait(lambda: self.status['mode']=='FAILED' and self.status['reason']=='takeoff_timeout'
                  and not self.status['owns_flight'],16,'arm denial deadline')
        assert not self.state.armed
        self.check('rejected_arming_is_not_takeoff_and_times_out')
        fake.allow_arm = True
        fake.fixed_position = True
        self.send('TAKEOFF')
        self.wait(lambda: self.status['mode']=='CLIMB',7,'climb timeout setup')
        self.wait(lambda: self.status['mode']=='FAILED' and self.status['reason']=='takeoff_timeout',
                  16,'height deadline')
        fake.fixed_position = False
        self.wait(lambda: not self.state.armed and not self.status['owns_flight'],
                  5,'automatic disarm following grounded takeoff timeout')
        self.check('height_timeout_cannot_report_takeoff')
        fake.mode = 'POSCTL'
        self.wait(lambda: self.state.mode=='POSCTL',2,'mode refusal initial mode')
        fake.reject_mode = True
        self.send('TAKEOFF')
        self.wait(lambda: self.status['mode']=='WAIT_OFFBOARD' and self.state.armed,7,'mode refusal setup')
        self.wait(lambda: self.status['mode']=='FAILED' and not self.state.armed
                  and not self.status['owns_flight'],16,'mode refusal deadline')
        fake.reject_mode = False
        self.check('mode_refusal_does_not_enter_climb')
        self.takeoff()
        self.begin()
        p,_ = decode(self.odom.pose.pose)
        self.plan(p,(p[0]+.8,p[1],p[2]))
        self.wait(lambda: self.status['mode']=='TRACK',3,'tracking for planner fault')
        self.emit_ready = False
        self.wait(lambda: self.status['context'] is None and self.status['mode']=='HOLD',
                  2,'planner heartbeat revoked')
        self.check('planner_disconnect_revokes_trajectory')
        fake.publish_odom = False
        self.wait(lambda: self.status['mode']=='FAILED' and not self.status['ready'],3,'stale odometry')
        self.wait(lambda: self.state.mode=='AUTO.LAND',3,'stale odometry landing request')
        fake.publish_odom = True
        self.wait(lambda: self.landed.landed_state==1,5,'ground feedback')
        self.wait(lambda: not self.state.armed and not self.status['owns_flight'],
                  5,'stale odometry recovery to disarmed ground')
        self.check('stale_odometry_enters_landing_failsafe')
        self.takeoff()
        fake.connected = False
        self.wait(lambda: self.status['mode']=='FAILED' and not self.status['ready'],3,'FCU disconnect')
        after = time.monotonic()
        time.sleep(.25)
        assert not any(p[0]>after+.08 for p in self.points)
        fake.connected = True
        self.wait(lambda: not self.state.armed and not self.status['owns_flight'],
                  9,'reconnected FCU reports safe ground state')
        self.check('link_loss_revokes_setpoint_stream')
        self.restart()

    def restart(self):
        old = self.status['flight_session_id']
        # Deliver the retained command only to the replacement process.
        subprocess.run(['rosnode','kill','/flight_executor'],check=True,
                       stdout=subprocess.DEVNULL, timeout=6)
        stale = rospy.Publisher('/uav/flight/command',String,queue_size=1,latch=True)
        stale.publish(String(data=json.dumps(dict(flight_session_id=old,seq=self.seq+101,
            command='TAKEOFF',height=1.6))))
        self.replacement = subprocess.Popen(['rosrun','uav_vln_bringup','flight_executor.py','_mock_fcu:=true'])
        self.wait(lambda: self.status['ready'] and self.status['flight_session_id']!=old,8,'new executor session')
        time.sleep(2.5)
        assert self.status['seq']==0 and not self.state.armed and not self.status['owns_flight']
        stale.unregister()
        self.check('restart_rejects_latched_previous_session_takeoff')

    def sitl_faults(self):
        faults = SitlFaultInputs()
        self.fault_inputs = faults
        pubs = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        assert not pubs.get('/mavros/setpoint_position/local')
        self.replacement = subprocess.Popen([
            'rosrun','uav_vln_bringup','flight_executor.py',
            '/mavros/state:=/uav/flight_test/state',
            '/mavros/local_position/odom:=/uav/flight_test/odom',
            '/mavros/cmd/arming:=/uav/flight_test/arming'])
        self.wait(lambda: self.status and self.status['ready'] and self.state and self.landed
                  and self.command.get_num_connections(),30,'SITL fault initial state')
        assert not self.state.armed and self.landed.landed_state==1

        if self.args.fault_case == 'arm':
            faults.refuse_arm = True
            self.send('TAKEOFF',height=1.6)
            self.wait(lambda: self.status['mode']=='WAIT_ARM',6,'SITL arm refusal')
            self.wait(lambda: self.status['reason']=='takeoff_timeout' and not self.status['owns_flight'],
                      16,'SITL arm deadline')
            assert not self.state.armed
            self.check('sitl_arming_service_refusal_no_takeoff')
            return

        if self.args.fault_case == 'height':
            faults.mask_height = True
            self.send('TAKEOFF',height=1.6)
            self.wait(lambda: self.status['mode']=='CLIMB' and self.state.armed,16,'SITL climb reached')
            self.wait(lambda: self.status['reason']=='takeoff_timeout',17,'SITL climb deadline')
            assert self.odom.pose.pose.position.z > .8
            self.wait(lambda: not self.state.armed and self.landed.landed_state==1
                      and not self.status['owns_flight'],25,'SITL height timeout landing')
            self.check('sitl_height_confirmation_timeout_lands')
            return

        self.takeoff()
        if self.args.fault_case == 'odom':
            faults.stale_stamp = True
            self.wait(lambda: self.status['mode']=='FAILED' and not self.status['ready'],3,'SITL stale stamp')
            self.wait(lambda: self.state.mode=='AUTO.LAND',8,'SITL stale stamp landing request')
            faults.stale_stamp = False
            self.wait(lambda: not self.state.armed and self.landed.landed_state==1
                      and not self.status['owns_flight'],25,'SITL stale stamp landing')
            self.check('sitl_stale_odom_stamp_lands')
            return

        faults.drop_link = True
        self.wait(lambda: self.status['mode']=='FAILED' and not self.status['ready'],4,'SITL link loss')
        after = time.monotonic()
        self.wait(lambda: self.state.mode!='OFFBOARD',10,'PX4 offboard loss failsafe')
        assert not any(p[0]>after+.08 for p in self.points)
        recovery_mode = self.state.mode
        faults.drop_link = False
        self.wait(lambda: not self.state.armed and self.landed.landed_state==1
                  and not self.status['owns_flight'],25,'SITL link recovery landing')
        self.check('sitl_feedback_link_loss_revokes_stream_and_px4_exits_offboard',
                   px4_mode_after_loss=recovery_mode)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mock',action='store_true')
    parser.add_argument('--sitl',action='store_true')
    parser.add_argument('--real-ego',action='store_true')
    parser.add_argument('--sitl-faults',action='store_true')
    parser.add_argument('--fault-case',choices=['arm','height','odom','link'])
    parser.add_argument('--yaw',type=float)
    parser.add_argument('--hold',type=float,default=10.)
    parser.add_argument('--out',type=Path,required=True)
    args = parser.parse_args()
    if args.mock == args.sitl:
        parser.error('select exactly one of --mock or --sitl')
    if args.sitl_faults and (not args.sitl or args.real_ego):
        parser.error('SITL faults require a separate SITL-only run')
    if args.sitl_faults != bool(args.fault_case):
        parser.error('select a single --fault-case for --sitl-faults')
    expected = 'http://127.0.0.1:11315' if args.mock else 'http://127.0.0.1:11311'
    if os.environ.get('ROS_MASTER_URI') != expected:
        parser.error('unexpected ROS master for this run')
    rospy.init_node('stage9_validation',anonymous=True)
    if args.sitl:
        from gazebo_msgs.srv import GetWorldProperties
        rospy.wait_for_service('/gazebo/get_world_properties',timeout=5)
        world = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)()
        assert world.success and 'iris_depth_camera' in world.model_names
    trial = Trial(args)
    fatal = None
    try:
        if args.sitl_faults:
            trial.sitl_faults()
        else:
            trial.run()
    except Exception as exc:
        fatal = repr(exc)
        print('FAIL',fatal,flush=True)
        if trial.status and trial.status['owns_flight']:
            try:
                trial.land()
            except Exception as stop_error:
                print('CLEANUP',repr(stop_error),flush=True)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'results.json').write_text(json.dumps(dict(passed=fatal is None, fatal=fatal,
        checks=trial.checks, fcu_states=trial.mode_history, flight_states=trial.flight_history,
        planning=trial.planner_results,
        final_armed=trial.state.armed if trial.state else None),indent=2)+'\n')
    (args.out/'trace.json').write_text(json.dumps(dict(setpoints=trial.points,odometry=trial.poses,
        fcu_targets=trial.targets,velocity_pairs=trial.velocity_pairs))+'\n')
    if trial.replacement is not None:
        trial.replacement.terminate()
        trial.replacement.wait(timeout=6)
    if trial.fault_inputs is not None:
        for subscriber in trial.fault_inputs.inputs:
            subscriber.unregister()
    if trial.fake is not None:
        trial.fake.done.set()
        trial.fake.worker.join(timeout=2)
    if fatal:
        raise SystemExit(1)


if __name__=='__main__':
    main()
