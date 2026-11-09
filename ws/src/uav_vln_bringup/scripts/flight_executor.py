#!/usr/bin/env python3
"""Single SITL flight command owner for the verified planning interface."""
import json
import math
import os
import threading
import time
import uuid

import rosgraph
import rospy
from ego_planner.msg import Bspline
from gazebo_msgs.srv import GetWorldProperties
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String

from navigation_core.bridge import BridgeDecisions, Feedback
from navigation_core.bspline import BSpline
from navigation_core.control import ControlArbiter, Reference
from navigation_core.trajectory import Sample, TimedTrajectory
from navigation_core.yaw import angle_delta, quaternion, quaternion_yaw


SETPOINT = '/mavros/setpoint_position/local'
PREVIEW = '/uav/goal_pose'


class FlightExecutor:
    def __init__(self):
        rospy.init_node('flight_executor')
        if rospy.get_param('~mock_fcu', False):
            if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11315':
                raise RuntimeError('Mock FCU requires the isolated test master')
        else:
            self.require_sitl()
        self.lock = threading.RLock()
        self.bridge = BridgeDecisions(goal_timeout=.35, max_pose_age=.5)
        self.control = ControlArbiter()
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.highest = {}
        self.planner = None
        self.planner_at = 0.
        self.gateway = None
        self.plan_deadline = None
        self.frame = None
        self.state = self.odom = self.landed = None
        self.owned = False
        self.graph = False
        self.graph_at = 0.
        self.actions = set()
        self.action_modes = {
            'ARM': 'WAIT_ARM', 'SET_OFFBOARD': 'WAIT_OFFBOARD',
            'AUTO.RTL': 'LANDING',
        }
        self.stop_id = None
        self.stop_reason = ''
        self.stable_since = None
        self.last_ros = None
        self.clock_advanced_at = time.monotonic()
        self.pub = rospy.Publisher(SETPOINT, PoseStamped, queue_size=1, tcp_nodelay=True)
        self.preview = rospy.Publisher(PREVIEW, PoseStamped, queue_size=1)
        self.status = rospy.Publisher('/uav/flight/status', String, queue_size=5, latch=True)
        self.cancel_pub = rospy.Publisher('/uav/cancel_planning', String, queue_size=5)
        rospy.Subscriber('/mavros/state', State, self.capture, 'state', queue_size=1)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self.capture, 'landed', queue_size=1)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.capture, 'odom', queue_size=1)
        rospy.Subscriber('/uav/flight/command', String, self.command, queue_size=20)
        rospy.Subscriber('/uav/planning/readiness', String, self.readiness, queue_size=1)
        rospy.Subscriber('/uav/planning/result', String, self.plan_result, queue_size=20)
        rospy.Subscriber('/uav/planning/trajectory', Bspline, self.spline, queue_size=10)
        threading.Thread(target=self.observe_graph, daemon=True).start()

    @staticmethod
    def require_sitl():
        if (not rospy.get_param('/use_sim_time', False)
                or rospy.get_param('/mavros/fcu_url', '') != 'udp://:14540@127.0.0.1:14580'):
            raise RuntimeError('The verified executor requires the project PX4 SITL endpoint')
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=5)
        world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
        if not world.success or 'iris_depth_camera' not in world.model_names:
            raise RuntimeError('The verified executor requires the simulated Iris model')

    def capture(self, msg, field):
        with self.lock:
            setattr(self, field, (msg, time.monotonic()))

    def observe_graph(self):
        master = rosgraph.Master(rospy.get_name())
        while not rospy.is_shutdown():
            try:
                pubs, subs, _ = master.getSystemState()
                command_topics = {topic for topic, nodes in subs
                                  if topic.startswith('/mavros/setpoint_') and '/mavros' in nodes}
                owners = dict(pubs)
                allowed = all(not (set(owners.get(topic, [])) - {rospy.get_name()})
                              for topic in command_topics | {SETPOINT, PREVIEW})
                allowed &= rospy.get_name() in owners.get(SETPOINT, [])
            except Exception as exc:
                rospy.logwarn_throttle(5, 'Cannot inspect flight publishers: %s', exc)
                allowed = False
            with self.lock:
                self.graph, self.graph_at = allowed, time.monotonic()
            time.sleep(.25)

    def feedback(self, wall, stamp):
        if not (self.state and self.odom and self.landed):
            return None
        state, state_at = self.state
        odom, odom_at = self.odom
        landed, landed_at = self.landed
        if (not state.connected or wall-state_at > 2.5 or wall-landed_at > 2.5
                or odom.header.frame_id != 'map' or odom.child_frame_id != 'base_link'
                or stamp <= 0. or odom.header.stamp.is_zero()
                or not -.1 <= stamp-odom.header.stamp.to_sec() <= .5
                or wall-self.clock_advanced_at > .5):
            return None
        if landed.landed_state not in (1, 2, 3, 4):
            return None
        try:
            p, q = odom.pose.pose.position, odom.pose.pose.orientation
            v = odom.twist.twist.linear
            return Feedback((p.x, p.y, p.z), quaternion_yaw((q.x, q.y, q.z, q.w)),
                            state.connected, state.armed, state.mode, landed.landed_state == 1,
                            wall-odom_at, math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z))
        except ValueError:
            return None

    def planner_ready(self, wall):
        return self.planner is not None and wall-self.planner_at < .7

    def readiness(self, msg):
        try:
            data = json.loads(msg.data)
            gateway, planner = data['gateway_session_id'], data['planner_session_id']
            ready = (gateway, planner) if data['schema_version'] == 1 and data['ready'] is True else None
            if ready and not all(isinstance(v, str) and v for v in ready):
                return
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            self.planner, self.planner_at = ready, time.monotonic()

    def revoke(self, feedback, reason, stop_id=None, cancel_planner=True):
        context = self.control.buffer.context
        if context and self.gateway and cancel_planner:
            mission, generation, _ = context
            self.cancel_pub.publish(String(data=json.dumps(dict(
                schema_version=1, mission_id=mission, through_goal_id=generation,
                gateway_session_id=self.gateway))))
        actual = Reference(feedback.position, feedback.yaw) if feedback else self.bridge.last_actual
        if actual is not None:
            self.control.cancel(actual)
        self.gateway = self.plan_deadline = None
        self.bridge.goal = self.bridge.goal_at = None
        self.stop_id, self.stop_reason, self.stable_since = stop_id, reason, None

    def hold(self, fb, reason, stop_id=None, cancel_planner=True):
        self.revoke(fb, reason, stop_id, cancel_planner)
        if self.bridge.mode in ('PRESTREAM', 'WAIT_ARM', 'WAIT_OFFBOARD', 'CLIMB'):
            self.bridge.command('LAND', fb, time.monotonic())
        elif self.bridge.mode in ('TRACK', 'HOLD'):
            self.bridge.command('HOVER', fb, time.monotonic())

    def command(self, msg):
        try:
            data = json.loads(msg.data)
            seq, command = data['seq'], data['command']
            if data.get('flight_session_id') != self.session or type(seq) is not int or seq <= 0:
                return
        except (ValueError, TypeError, KeyError, AttributeError):
            return
        with self.lock:
            if seq <= self.sequence:
                return
            self.sequence = seq
            wall = time.monotonic()
            fb = self.feedback(wall, rospy.Time.now().to_sec())
            if not fb or fb.pose_age > .5:
                return
            if command == 'LAND' and self.owned:
                self.revoke(fb, 'land_requested', seq)
                self.bridge.command('LAND', fb, wall)
            elif command == 'HOVER' and self.owned and self.bridge.mode != 'LANDING':
                self.hold(fb, 'hold_requested', seq)
            elif command == 'TAKEOFF' and not self.owned and not fb.armed:
                height = data.get('height', 1.5)
                if self.bridge.mode == 'COMPLETE':
                    self.bridge.mode = 'IDLE'
                if (self.graph and wall-self.graph_at < 1. and self.pub.get_num_connections()
                        and type(height) in (int, float) and .8 <= height <= 2.3
                        and self.bridge.command(command, fb, wall, height=height)):
                    self.owned = True
                    self.stop_id = self.stable_since = None
                    self.stop_reason = ''
            elif command == 'BEGIN' and self.owned and self.bridge.mode == 'HOLD':
                mission, goal = data.get('mission_id'), data.get('goal_id')
                gateway, planner = data.get('gateway_session_id'), data.get('planner_session_id')
                if (not fb.armed or fb.mode != 'OFFBOARD' or not self.planner_ready(wall)
                        or self.planner != (gateway, planner)
                        or not isinstance(mission, str) or not mission
                        or type(goal) is not int or goal <= self.highest.get((gateway, mission), 0)):
                    return
                self.highest[(gateway, mission)] = goal
                self.control.begin(mission, goal, planner, self.bridge.anchor)
                self.gateway, self.plan_deadline = gateway, wall+8.
                self.stop_id = self.stable_since = None
                self.stop_reason = ''

    def plan_result(self, msg):
        try:
            data = json.loads(msg.data)
            context = (data['mission_id'], data['goal_id'], data['planner_session_id'])
            if data['schema_version'] != 1:
                return
            status = data['status']
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            if (not self.control.live or context != self.control.buffer.context
                    or data.get('gateway_session_id') != self.gateway):
                return
            if status in ('CANCELLING', 'CANCELLED', 'CANCEL_TIMEOUT', 'MAP_NOT_READY',
                          'FAILED', 'REJECTED', 'SESSION_CHANGED', 'STOP'):
                fb = self.feedback(time.monotonic(), rospy.Time.now().to_sec())
                if fb:
                    self.hold(fb, 'planner_'+status.lower(), cancel_planner=False)

    def spline(self, msg):
        with self.lock:
            context = (msg.mission_id, msg.goal_id, msg.planner_session_id)
            if not self.control.live or self.control.buffer.context != context:
                return
            now = rospy.Time.now().to_sec()
            wall = time.monotonic()
            fb = self.feedback(wall, now)
            if (not fb or not fb.armed or fb.mode != 'OFFBOARD' or not self.planner_ready(wall)
                    or self.planner != (self.gateway, context[2])):
                return
            if msg.emergency_stop:
                self.hold(fb, 'ego_stop')
                return
            if msg.traj_id <= self.control.buffer.last_id or len(msg.pos_pts) > 512 or msg.order != 3:
                return
            try:
                curve = BSpline([(p.x, p.y, p.z) for p in msg.pos_pts], msg.knots, msg.order)
                start = msg.start_time.to_sec()
                if not 0. < curve.duration <= 60. or not 0. < start <= now+.5 or now >= start+curve.duration:
                    return
                yaw = self.bridge.anchor.yaw
                points = tuple(Sample(p, v, yaw, t) for t, p, v, _ in curve.samples(.04))
                traj = TimedTrajectory(*context, msg.traj_id, start, points)
                first = traj.sample(now)
                ref = self.control.output(now) or self.bridge.anchor
                if math.dist(first.position, ref.position) > .6 or math.dist(first.position, fb.position) > .75:
                    self.hold(fb, 'discontinuous_trajectory')
                    return
                if self.control.track(traj, now):
                    self.plan_deadline = None
            except (ValueError, TypeError, IndexError) as exc:
                rospy.logwarn_throttle(2, 'Rejecting invalid spline: %s', exc)

    def action(self, action):
        family = 'arming' if action in ('ARM', 'DISARM') else 'mode'
        if family in self.actions:
            return
        self.actions.add(family)

        def send():
            try:
                with self.lock:
                    if action == 'AUTO.LAND':
                        allowed = self.bridge.mode in ('LANDING', 'FAILED')
                    elif action == 'DISARM':
                        fb = self.feedback(time.monotonic(), rospy.Time.now().to_sec())
                        allowed = self.bridge.mode in ('LANDING', 'FAILED') and fb and fb.landed and fb.pose_age <= .5
                    else:
                        allowed = self.bridge.mode == self.action_modes[action]
                    if not self.owned or not allowed:
                        return
                if family == 'arming':
                    response = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)(action == 'ARM')
                    accepted = response.success
                else:
                    mode = 'OFFBOARD' if action == 'SET_OFFBOARD' else action
                    response = rospy.ServiceProxy('/mavros/set_mode', SetMode)(custom_mode=mode)
                    accepted = response.mode_sent
                if not accepted:
                    rospy.logwarn_throttle(1, 'MAVROS rejected %s', action)
            except (rospy.ServiceException, rospy.ROSException) as exc:
                rospy.logwarn_throttle(1, 'MAVROS service %s: %s', action, exc)
            finally:
                with self.lock:
                    self.actions.discard(family)

        threading.Thread(target=send, daemon=True).start()

    def run(self):
        next_status = 0.
        while not rospy.is_shutdown():
            wall = time.monotonic()
            with self.lock:
                stamp = rospy.Time.now()
                now = stamp.to_sec()
                clock_reset = self.last_ros is not None and now < self.last_ros
                if self.last_ros != now:
                    self.clock_advanced_at = wall
                self.last_ros = now
                fb = self.feedback(wall, now)
                healthy = fb is not None and fb.pose_age <= .5 and self.graph and wall-self.graph_at < 1.
                if self.owned and (not healthy or clock_reset):
                    if self.bridge.mode != 'FAILED':
                        self.revoke(fb, 'clock_reset' if clock_reset else 'feedback_or_owner_lost')
                        self.bridge._fail(self.stop_reason)
                if self.owned and self.bridge.mode == 'FAILED':
                    # Give PX4 a landing request when feedback permits. With no
                    # setpoint stream PX4 can also apply its own offboard failsafe.
                    if self.state and self.state[0].connected and self.state[0].armed:
                        if fb and fb.landed and fb.pose_age <= .5:
                            for action in self.bridge._action('DISARM', wall):
                                self.action(action)
                        elif self.state[0].mode != 'AUTO.LAND':
                            for action in self.bridge._action('AUTO.LAND', wall):
                                self.action(action)
                    elif fb and fb.landed and not self.actions:
                        self.owned = False
                elif self.owned and healthy:
                    ctx = self.control.buffer.context
                    if ctx and (not self.planner_ready(wall) or self.planner != (self.gateway, ctx[2])
                                or self.plan_deadline is not None and wall > self.plan_deadline):
                        self.hold(fb, 'planning_heartbeat_or_deadline')
                    if self.control.mode == 'TRACK':
                        ref = self.control.output(now)
                        if self.control.mode == 'TRACK' and ref:
                            self.bridge.set_goal(ref, wall, fb)
                            self.bridge.command('GOTO', fb, wall)
                        else:
                            self.hold(fb, 'playback_invalidated')
                    if self.bridge.mode in ('PRESTREAM', 'WAIT_ARM', 'WAIT_OFFBOARD') and fb.landed:
                        # The cold-start estimator can settle while PX4 is
                        # still preparing. Freeze the takeoff anchor at climb.
                        height = self.bridge.anchor.position[2]
                        self.bridge.anchor = Reference((*fb.position[:2], height), fb.yaw)
                    decision = self.bridge.tick(fb, wall)
                    if decision.mode == 'COMPLETE' and self.actions:
                        self.bridge.mode = 'LANDING'
                    if decision.mode == 'FAILED':
                        self.revoke(fb, decision.reason)
                    for action in decision.actions:
                        self.action(action)
                    if decision.mode == 'COMPLETE' and 'arming' not in self.actions and 'mode' not in self.actions:
                        self.owned = False
                    if (decision.reference and decision.mode not in ('COMPLETE', 'FAILED')
                            and (decision.mode != 'LANDING' or fb.mode == 'OFFBOARD')):
                        ref = decision.reference
                        pose = PoseStamped(header=Header(stamp=stamp, frame_id='map'))
                        pose.pose.position = Point(*ref.position)
                        pose.pose.orientation = Quaternion(*quaternion(ref.yaw))
                        self.pub.publish(pose)
                        self.preview.publish(pose)
                if self.stop_reason and self.bridge.mode == 'HOLD' and healthy and fb.armed and fb.mode == 'OFFBOARD':
                    anchor = self.bridge.anchor
                    if (math.dist(fb.position, anchor.position) <= .3 and fb.speed <= .15
                            and abs(angle_delta(fb.yaw, anchor.yaw)) <= math.radians(10)):
                        self.stable_since = wall if self.stable_since is None else self.stable_since
                    else:
                        self.stable_since = None
                else:
                    self.stable_since = None
                if wall >= next_status:
                    next_status = wall+.08
                    active = self.control.buffer.active
                    self.status.publish(String(data=json.dumps(dict(
                        flight_session_id=self.session, seq=self.sequence, mode=self.bridge.mode,
                        ready=healthy, owner=self.graph, reason=self.bridge.reason or self.stop_reason,
                        context=self.control.buffer.context, trajectory_id=active.traj_id if active else None,
                        stop_id=self.stop_id, stop_confirmed=self.stable_since is not None
                        and wall-self.stable_since >= 1., owns_flight=self.owned))))
            time.sleep(max(0., .04-(time.monotonic()-wall)))


if __name__ == '__main__':
    FlightExecutor().run()
