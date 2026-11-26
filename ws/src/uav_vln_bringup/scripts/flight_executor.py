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
from tf.transformations import quaternion_matrix

from navigation_core.bridge import BridgeDecisions, Feedback
from navigation_core.bspline import BSpline
from navigation_core.contracts import finite
from navigation_core.control import ControlArbiter, Reference
from navigation_core.trajectory import Sample, TimedTrajectory
from navigation_core.yaw import YawPolicy, angle_delta, quaternion, quaternion_yaw


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
        self.mission_ack_deadline = None
        self.fence = (rospy.get_param('/uav/segment_config/fence')
                      if rospy.get_param('~segment_fence',False) else None)
        self.state = self.odom = self.landed = None
        self.previous_ground_stamp = None
        self.ground_after_stamp = 0.
        self.disarmed_landing_stamp = None
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
        self.turn = None
        self.last_ros = None
        self.clock_advanced_at = time.monotonic()
        self.pub = rospy.Publisher(SETPOINT, PoseStamped, queue_size=1, tcp_nodelay=True)
        self.preview = rospy.Publisher(PREVIEW, PoseStamped, queue_size=1)
        self.status = rospy.Publisher('/uav/flight/status', String, queue_size=5, latch=True)
        self.cancel_pub = rospy.Publisher('/uav/cancel_planning', String, queue_size=5)
        self.trajectory_ack = rospy.Publisher('/uav/flight/trajectory_ack', String, queue_size=20)
        self.handover_accel = rospy.get_param('~handover_accel_limit', .8)
        if not finite(self.handover_accel) or self.handover_accel <= 0:
            raise ValueError('positive handover acceleration limit required')
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
            wall = time.monotonic()
            if field == 'landed':
                self.previous_ground_stamp = (
                    self.landed[0].header.stamp.to_sec()
                    if self.landed and self.landed[0].landed_state == 1 else None)
            elif field == 'odom':
                v = msg.twist.twist.linear
                if not math.hypot(v.x, v.y, v.z) <= .15:
                    self.ground_after_stamp = max(self.ground_after_stamp, msg.header.stamp.to_sec())
            elif field == 'state':
                if self.bridge.mode != 'LANDING' or not self.owned or not msg.connected or msg.armed:
                    self.disarmed_landing_stamp = None
                elif self.disarmed_landing_stamp is None or not self.state or wall-self.state[1] > 2.5:
                    self.disarmed_landing_stamp = rospy.Time.now().to_sec()
            setattr(self, field, (msg, wall))

    def request_landing(self, fb, wall):
        if self.bridge.mode != 'LANDING':
            self.ground_after_stamp = rospy.Time.now().to_sec()
            self.disarmed_landing_stamp = None
        self.bridge.command('LAND', fb, wall)

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

    def feedback(self, wall, stamp, *, confirm_disarmed_landing=False):
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
            # Commands retain the motion guard. Only a completed motor shutdown
            # can use two new FCU ground reports despite residual odometry speed.
            ground_stamp = landed.header.stamp.to_sec()
            ground_after = self.ground_after_stamp
            if (confirm_disarmed_landing and self.owned and self.bridge.mode == 'LANDING'
                    and not state.armed and self.disarmed_landing_stamp is not None):
                ground_after = min(ground_after, self.disarmed_landing_stamp)
            grounded = (landed.landed_state == 1 and self.previous_ground_stamp is not None
                        and ground_after < self.previous_ground_stamp < ground_stamp
                        and ground_stamp-self.previous_ground_stamp <= 2.5
                        and -.1 <= stamp-ground_stamp <= 1.5)
            return Feedback((p.x, p.y, p.z), quaternion_yaw((q.x, q.y, q.z, q.w)),
                            state.connected, state.armed, state.mode, grounded,
                            wall-odom_at, math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z))
        except ValueError:
            return None

    def inside_fence(self, position, margin=0.):
        return not self.fence or all(
            self.fence[axis+'_min']-margin<=value<=self.fence[axis+'_max']+margin
            for axis,value in zip('xyz',position))

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
        self.mission_ack_deadline = None
        self.bridge.goal = self.bridge.goal_at = None
        self.stop_id, self.stop_reason, self.stable_since = stop_id, reason, None
        self.turn = None

    def hold(self, fb, reason, stop_id=None, cancel_planner=True):
        new_stop = stop_id is not None and stop_id != self.stop_id
        self.revoke(fb, reason, stop_id, cancel_planner)
        if self.bridge.mode in ('PRESTREAM', 'WAIT_ARM', 'WAIT_OFFBOARD', 'CLIMB'):
            self.request_landing(fb, time.monotonic())
        elif self.bridge.mode in ('TRACK', 'HOLD'):
            # A distinct stop command creates one new hold anchor, even if
            # already holding after a turn. Ordinary feedback never moves it.
            if new_stop:
                self.bridge.anchor = Reference(fb.position, fb.yaw)
            self.bridge.command('HOVER', fb, time.monotonic())

    def command(self, msg):
        try:
            data = json.loads(msg.data)
            command = data['command']
            if data.get('flight_session_id') != self.session:
                return
            if command == 'ACK_STATUS':
                self.acknowledge_status(data, time.monotonic())
                return
            seq = data['seq']
            if type(seq) is not int or seq <= 0:
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
                self.request_landing(fb, wall)
            elif command == 'HOVER' and self.owned and self.bridge.mode != 'LANDING':
                self.hold(fb, 'hold_requested', seq)
            elif command == 'TAKEOFF' and not self.owned and not fb.armed:
                height = data.get('height', 1.5)
                if self.bridge.mode == 'COMPLETE':
                    self.bridge.mode = 'IDLE'
                if (self.graph and wall-self.graph_at < 1. and self.pub.get_num_connections()
                        and type(height) in (int, float) and .8 <= height <= 2.3
                        and self.bridge.command(command, fb, wall, height=height)):
                    self.ground_after_stamp = rospy.Time.now().to_sec()
                    self.owned = True
                    self.stop_id = self.stable_since = None
                    self.stop_reason = ''
            elif command == 'TURN' and self.owned and self.bridge.mode == 'HOLD':
                target, via = data.get('yaw'), data.get('via')
                if (not fb.armed or fb.mode != 'OFFBOARD' or self.control.live
                        or self.turn is not None or self.stable_since is None or wall-self.stable_since<1.
                        or not finite(target) or abs(target)>math.pi
                        or via is not None and (not finite(via) or abs(via)>math.pi)):
                    return
                headings = [target] if via is None else [via, target]
                self.turn = (headings, YawPolicy(self.bridge.anchor.yaw, rate_limit=.65), wall)
                self.stop_id, self.stop_reason, self.stable_since = seq, 'turn_requested', None
            elif command == 'BEGIN' and self.owned and self.bridge.mode == 'HOLD':
                if self.turn is not None:
                    return
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
                self.mission_ack_deadline = wall+.7 if data.get('require_status_ack') is True else None
                self.stop_id = self.stable_since = None
                self.stop_reason = ''

    def acknowledge_status(self, data, wall):
        with self.lock:
            stamp = data.get('status_at')
            if (self.mission_ack_deadline is not None and wall <= self.mission_ack_deadline
                    and data.get('context') == list(self.control.buffer.context)
                    and finite(stamp) and wall-.7 < stamp <= wall):
                # Both times originate from this executor's monotonic clock.
                # Replaying a queued acknowledgement cannot extend the lease.
                self.mission_ack_deadline = max(self.mission_ack_deadline, stamp+.7)

    def check_plan_authority(self, fb, wall):
        ctx = self.control.buffer.context
        if ctx and self.mission_ack_deadline is not None and wall > self.mission_ack_deadline:
            self.hold(fb, 'mission_feedback_timeout')
        elif ctx and (not self.planner_ready(wall) or self.planner != (self.gateway, ctx[2])
                      or self.plan_deadline is not None and wall > self.plan_deadline):
            self.hold(fb, 'planning_heartbeat_or_deadline')

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
                    reason=data.get('reason')
                    suffix=reason if reason in ('unknown_trajectory','outside_known_map','occupied_trajectory') else status.lower()
                    self.hold(fb, 'planner_'+suffix, cancel_planner=False)

    def spline(self, msg):
        with self.lock:
            context = (msg.mission_id, msg.goal_id, msg.planner_session_id)
            def acknowledge(accepted, reason, **metrics):
                self.trajectory_ack.publish(String(data=json.dumps(dict(
                    flight_session_id=self.session, mission_id=msg.mission_id,
                    goal_id=msg.goal_id, planner_session_id=msg.planner_session_id,
                    traj_id=msg.traj_id, accepted=accepted, reason=reason, **metrics))))
            if not self.control.live or self.control.buffer.context != context:
                acknowledge(False, 'wrong_context')
                return
            now = rospy.Time.now().to_sec()
            wall = time.monotonic()
            fb = self.feedback(wall, now)
            if (not fb or not fb.armed or fb.mode != 'OFFBOARD' or not self.planner_ready(wall)
                    or self.planner != (self.gateway, context[2])):
                acknowledge(False, 'execution_unavailable')
                return
            if msg.emergency_stop:
                self.hold(fb, 'ego_stop')
                acknowledge(True, 'emergency_stop')
                return
            if msg.traj_id <= self.control.buffer.last_id:
                acknowledge(False, 'old_trajectory')
                return
            if len(msg.pos_pts) > 512 or msg.order != 3:
                acknowledge(False, 'invalid_spline')
                return
            try:
                curve = BSpline([(p.x, p.y, p.z) for p in msg.pos_pts], msg.knots, msg.order)
                start = msg.start_time.to_sec()
                if not 0. < curve.duration <= 60. or not 0. < start <= now+.5 or now >= start+curve.duration:
                    acknowledge(False, 'invalid_time')
                    return
                yaw = self.bridge.anchor.yaw
                points = tuple(Sample(p, v, yaw, t) for t, p, v, _ in curve.samples(.04))
                if any(not self.inside_fence(point.position) for point in points):
                    self.hold(fb, 'trajectory_outside_fence')
                    acknowledge(False, 'trajectory_outside_fence')
                    return
                traj = TimedTrajectory(*context, msg.traj_id, start, points)
                first = traj.sample(now)
                ref = self.control.output(now) or self.bridge.anchor
                if math.dist(first.position, ref.position) > .6 or math.dist(first.position, fb.position) > .75:
                    self.hold(fb, 'discontinuous_trajectory')
                    acknowledge(False, 'position_discontinuity')
                    return
                odom = self.odom[0]
                v, q = odom.twist.twist.linear, odom.pose.pose.orientation
                world_velocity = quaternion_matrix((q.x, q.y, q.z, q.w))[:3,:3].dot((v.x, v.y, v.z))
                # Compare at the takeover instant; allow acceleration between the
                # measured velocity timestamp and the first new reference.
                takeover = max(start, now)
                tolerance = .15 + self.handover_accel * abs(takeover - odom.header.stamp.to_sec())
                error = math.dist(first.velocity, world_velocity)
                metrics = dict(velocity_error_mps=error, velocity_tolerance_mps=tolerance)
                if not error <= tolerance:
                    self.hold(fb, 'discontinuous_trajectory')
                    acknowledge(False, 'velocity_discontinuity', **metrics)
                    return
                if self.control.track(traj, now):
                    self.plan_deadline = None
                    acknowledge(True, 'accepted', **metrics)
                else:
                    acknowledge(False, self.control.buffer.reason)
            except (ValueError, TypeError, IndexError) as exc:
                acknowledge(False, 'invalid_spline')
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
            with self.lock:
                wall = time.monotonic()
                stamp = rospy.Time.now()
                now = stamp.to_sec()
                clock_reset = self.last_ros is not None and now < self.last_ros
                if self.last_ros != now:
                    self.clock_advanced_at = wall
                self.last_ros = now
                fb = self.feedback(wall, now, confirm_disarmed_landing=True)
                healthy = fb is not None and fb.pose_age <= .5 and self.graph and wall-self.graph_at < 1.
                previous_mode = self.bridge.mode
                if (self.owned and healthy and self.bridge.mode in ('HOLD','TRACK')
                        and not self.inside_fence(fb.position,margin=.25)):
                    self.revoke(fb,'measured_pose_outside_fence')
                    self.bridge._fail(self.stop_reason)
                if self.owned and (not healthy or clock_reset):
                    if self.bridge.mode != 'FAILED':
                        self.revoke(fb, 'clock_reset' if clock_reset else 'feedback_or_owner_lost')
                        self.bridge._fail(self.stop_reason)
                if self.owned and self.bridge.mode == 'FAILED':
                    if previous_mode != 'FAILED':
                        self.ground_after_stamp = now
                        fb = self.feedback(wall, now)
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
                    self.check_plan_authority(fb, wall)
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
                    if self.turn is not None:
                        headings, policy, updated = self.turn
                        yaw = policy.update(headings[0], min(.1, wall-updated))
                        self.bridge.anchor = Reference(self.bridge.anchor.position, yaw)
                        if abs(angle_delta(headings[0], yaw)) < 1e-6:
                            headings.pop(0)
                        self.turn = (headings, policy, wall) if headings else None
                    decision = self.bridge.tick(fb, wall)
                    if decision.mode == 'COMPLETE' and self.actions:
                        self.bridge.mode = 'LANDING'
                    if decision.mode == 'FAILED':
                        self.ground_after_stamp = now
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
                if (self.stop_reason and self.turn is None and self.bridge.mode == 'HOLD'
                        and healthy and fb.armed and fb.mode == 'OFFBOARD'):
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
                        flight_session_id=self.session, seq=self.sequence, mode=self.bridge.mode, status_at=wall,
                        ready=healthy, owner=self.graph, reason=self.bridge.reason or self.stop_reason,
                        yaw_reference=(self.bridge.goal if self.bridge.mode=='TRACK' else
                                       self.bridge.anchor).yaw if self.bridge.anchor else None,
                        context=self.control.buffer.context, trajectory_id=active.traj_id if active else None,
                        stop_id=self.stop_id, stop_confirmed=self.stable_since is not None
                        and wall-self.stable_since >= 1., owns_flight=self.owned))))
            time.sleep(max(0., .04-(time.monotonic()-wall)))


if __name__ == '__main__':
    FlightExecutor().run()
