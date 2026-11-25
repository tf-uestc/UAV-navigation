#!/usr/bin/env python3
"""Exercise project planning with the verified flight owner and a simulated Iris."""
import argparse
from bisect import bisect_left
import json
import math
import os
from pathlib import Path
import time

from ego_planner.msg import GoalStatus
from gazebo_msgs.srv import GetLinkState, SpawnModel
from geometry_msgs.msg import Point, Pose
import rospy
import rosgraph
from std_msgs.msg import String

from validate_segments import Trial, decode


BODY = 'iris_depth_camera::iris::base_link'


class PlanningTrial(Trial):
    def __init__(self, args):
        super().__init__(args)
        rospy.Subscriber('/uav/ego/status', GoalStatus, self.ego_status, queue_size=25)
        rospy.Subscriber('/uav/flight/trajectory_ack', String, self.json_cb, 'trajectory_ack', queue_size=40)
        self.world = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    def ego_status(self, msg):
        if msg.mission_id:
            self.record('ego_status', dict(mission_id=msg.mission_id, goal_id=msg.goal_id,
                                          status=msg.status, reason=msg.reason))

    def configuration(self):
        grid=rospy.get_param('/ego_planner_node/grid_map')
        fence=rospy.get_param('/uav/segment_config/fence')
        bounds=((-grid['map_size_x']/2,grid['map_size_x']/2),
                (-grid['map_size_y']/2,grid['map_size_y']/2),
                (grid['ground_height'],min(grid['ground_height']+grid['map_size_z'],grid['virtual_ceil_height'])))
        assert all(lo < fence[axis+'_min'] < fence[axis+'_max'] < hi
                   for axis,(lo,hi) in zip('xyz',bounds)),(bounds,fence)
        assert grid['enable_independent_cloud'] is False and grid['pose_type']==1
        assert rospy.get_param('/ego_planner_node/fsm/require_known_space') is True
        subs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[1])
        assert '/ego_planner_node' not in subs.get('/grid_map/cloud',[])
        assert all('/ego_planner_node' in subs[topic] for topic in ('/uav/planning/depth','/uav/planning/camera_pose'))
        self.check('live_fence_inside_map_and_ceiling',grid_bounds=bounds,fence=fence,grid_parameters=grid)

    def takeoff(self):
        self.initial()
        start, _ = decode(self.odom.pose.pose)
        mission = self.new('takeoff', takeoff=True, waypoint=(*start[:2], 1.6))
        result = self.wait_phase(mission, ('DONE', 'FAILED'), 42)
        assert result['phase'] == 'DONE', result

    def track_metrics(self, mission, t0, t1):
        records = [(kind, data) for t, kind, data in self.trace if t0 <= t <= t1]
        odom = [d for kind, d in records if kind == 'odom']
        target = sorted((d for kind, d in records if kind == 'setpoint'), key=lambda d: d['stamp'])
        stamps = [d['stamp'] for d in target]
        errors = []
        for sample in odom:
            j = bisect_left(stamps, sample['stamp'])
            nearest = min(target[max(0, j - 1):j + 1],
                          key=lambda d: abs(d['stamp'] - sample['stamp']), default=None)
            if nearest and abs(nearest['stamp'] - sample['stamp']) <= .08:
                errors.append(math.dist(sample['position'], nearest['position']))
        tracking = [d for kind, d in records if kind == 'flight' and d['mode'] == 'TRACK'
                    and d['context'] and d['context'][0] == mission]
        assert odom and errors and tracking
        dispositions = [d for kind, d in records if kind == 'trajectory_ack' and d['mission_id'] == mission]
        expected = {(d['goal_id'],d['traj_id']) for kind,d in records
                    if kind=='spline' and d['mission_id']==mission}
        assert expected and expected <= {(d['goal_id'],d['traj_id']) for d in dispositions}, 'missing disposition'
        accepted = {(d['goal_id'],d['traj_id']) for d in dispositions if d['accepted']}
        assert {(d['context'][1],d['trajectory_id']) for d in tracking} <= accepted, 'executed without acknowledgement'
        assert all(-6 <= d['position'][0] <= 6 and -4 <= d['position'][1] <= 4
                   and 1.1 <= d['position'][2] <= 2.35 for d in odom), 'measured fence breach'
        endpoint, _ = decode(self.odom.pose.pose)
        twist = self.odom.twist.twist.linear
        return dict(peak_tracking_error_m=max(errors),
                    rmse_tracking_m=math.sqrt(sum(x*x for x in errors)/len(errors)),
                    odom_samples=len(odom), aligned_samples=len(errors),
                    executed_trajectories=sorted({(d['context'][1], d['trajectory_id']) for d in tracking}),
                    trajectory_dispositions=dispositions, endpoint=endpoint,
                    endpoint_speed_mps=math.hypot(twist.x, twist.y, twist.z))

    def move(self, label, goal, seconds=130):
        t0 = time.monotonic()
        mission = self.new(label, waypoint=goal)
        result = self.wait_phase(mission, ('DONE', 'FAILED'), seconds)
        assert result['phase'] == 'DONE' and result['segment_count'] > 0, result
        details = self.track_metrics(mission, t0, time.monotonic())
        assert math.dist(details['endpoint'], goal) <= .23, details
        assert details['endpoint_speed_mps'] <= .15, details
        self.check(label, segments=result['segment_count'], **details)
        return result

    def face_route(self, goal):
        # The injected observation sets the heading only. The subsequent manual
        # waypoint must be planned against real camera depth and actual odometry.
        start, yaw = decode(self.odom.pose.pose)
        heading = math.atan2(goal[1]-start[1],goal[0]-start[0])
        if abs(math.atan2(math.sin(heading-yaw),math.cos(heading-yaw))) < math.radians(5):
            return
        # Route fixtures run with the cloud detector disabled.
        standoff = rospy.get_param('/segment_mission/standoff',2.4)
        target = (start[0]+standoff*math.cos(heading),start[1]+standoff*math.sin(heading),.6)
        mission = self.new('fixture_set_heading',target=target)
        result = self.wait_phase(mission, ('DONE','FAILED'), 52.)
        assert result['phase']=='DONE' and result['segment_count']==0, result
        end, actual_yaw = decode(self.odom.pose.pose)
        assert math.dist(start,end)<.3
        assert abs(math.atan2(math.sin(heading-actual_yaw),math.cos(heading-actual_yaw)))<math.radians(10)
        self.check('fixture_set_heading_in_place', heading_degrees=math.degrees(heading))

    def route_matrix(self, repetitions, scenario):
        self.takeoff()
        for i in range(repetitions):
            start, _ = decode(self.odom.pose.pose)
            delta = .9 if start[0] < 0.5 else -.9
            if scenario in ('routes','straight'):
                goal = (start[0]+delta, start[1], 1.6)
                self.face_route(goal)
                self.move('straight', goal, 35)
            if scenario not in ('routes','outside'):
                continue
            invalid = self.new('outside', waypoint=(6.5, 0., 1.6))
            result = self.wait_phase(invalid, ('DONE', 'FAILED'), 15)
            assert result['phase'] == 'FAILED' and result['reason'] == 'outside_fence', result
            assert not any(k == 'plan_request' and d['mission_id'] == invalid for _, k, d in self.trace)
            assert self.flight['mode'] == 'HOLD' and self.flight['context'] is None
            self.check('outside_fence', repetition=i + 1)
        if scenario not in ('routes','long'):
            return
        position = (-4.2, -1.35, 1.6)
        self.face_route(position)
        self.move('position_for_long', position, 100)
        for i in range(repetitions):
            x = 4.2 if i % 2 == 0 else -4.2
            start, _ = decode(self.odom.pose.pose)
            goal = (x, -1.35, 1.6)
            distance = math.dist(start, goal)
            assert distance > 7.5
            self.face_route(goal)
            result = self.move('beyond_planning_horizon', goal)
            assert result['segment_count'] >= 5

    def dynamic_box(self, blocked=False):
        self.takeoff()
        ground = self.world(BODY, 'world')
        assert ground.success
        p = ground.link_state.pose.position
        center = p.x + 1.14, p.y
        # The wall extends beyond both lateral fence edges and the ceiling.
        size = .16, (22. if blocked else .36), (6. if blocked else 2.6)
        geometry = '<geometry><box><size>%s %s %s</size></box></geometry>' % size
        sdf = ('<sdf version="1.6"><model name="dynamic_test_box"><static>true</static>'
               '<link name="body"><collision name="collision">'+geometry+'</collision>'
               '<visual name="visual">'+geometry+'</visual></link></model></sdf>')
        model = Pose(position=Point(*center, size[2]/2))
        model.orientation.w = 1.
        start, _ = decode(self.odom.pose.pose)
        t0 = time.monotonic()
        mission = self.new('dynamic', waypoint=(start[0]+1.79, start[1], 1.6))
        self.wait_phase(mission, ('TRACK',), 15)
        first_id = self.flight['trajectory_id']
        self.wait(lambda: decode(self.odom.pose.pose)[0][0] > start[0]+.12, 7, 'started moving')
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        assert spawn('dynamic_test_box', sdf, '', model, 'world').success
        inserted = time.monotonic()
        self.record('obstacle_inserted', dict(center=center, size=size, first_trajectory_id=first_id))
        samples = []
        while time.monotonic() - inserted < 90.:
            world = self.world(BODY, 'world')
            assert world.success
            point = world.link_state.pose.position
            samples.append((point.x, point.y, point.z))
            if self.mission['mission_id'] == mission and self.mission['phase'] in ('DONE', 'FAILED'):
                break
            time.sleep(.06)
        self.record('dynamic_box_world_track', dict(samples=samples))
        clearance = min(math.hypot(max(0., abs(q[0]-center[0])-size[0]/2),
                                   max(0., abs(q[1]-center[1])-size[1]/2)) for q in samples)
        self.check('dynamic_box_clearance', minimum_center_to_box_surface_m=clearance)
        assert clearance > .24, clearance
        statuses = [d for t, k, d in self.trace if t >= inserted and k == 'ego_status'
                    and d['mission_id'] == mission]
        details = self.track_metrics(mission, t0, time.monotonic())
        goals = details['executed_trajectories']
        if blocked:
            assert self.mission['mission_id']==mission and self.mission['phase']=='FAILED', self.mission
            reasons = [d['reason'] for d in statuses]
            assert any(r in ('collision','occupied_trajectory','invalid_or_occupied_goal') for r in reasons), reasons
            self.wait(lambda: self.flight['mode']=='HOLD' and self.flight['stop_confirmed']
                      and self.flight['context'] is None, 10., 'no-route measured stop')
            assert all(q[0]<center[0]-size[0]/2 for q in samples)
            self.check('dynamic_wall_stopped',reason=self.mission['reason'],ego_reasons=reasons,**details)
            return
        continuous = (any(d['status']=='REPLANNED' and d['reason']=='collision' for d in statuses)
                      and any(goal==1 and tid>first_id for goal, tid in goals))
        guarded = any(d['status']=='FAILED' and d['reason'] in ('occupied_trajectory','unknown_trajectory','outside_known_map') for d in statuses)
        stopped = guarded or any(d['status']=='EMERGENCY_STOP' and d['reason']=='collision' for d in statuses)
        if stopped:
            stages = [(t, d) for t, k, d in self.trace if k=='mission' and d['mission_id']==mission]
            stopped_at = next(t for t, d in stages if d['phase']=='SETTLE' and d['reason'] in ('planner_stop','occupied_trajectory','unknown_trajectory','outside_known_map'))
            resumed = [(t,d) for t,d in stages if t>stopped_at and d['phase']=='TRACK'
                       and d['goal_id']>1 and d.get('detour_active')]
            assert resumed, dict(terminal=self.mission,ego_status=statuses[-5:])
            resumed_at=resumed[0][0]
            assert any(k=='flight' and stopped_at <= t < resumed_at and d['stop_confirmed']
                       and d['mode']=='HOLD' and d['context'] is None for t, k, d in self.trace)
            assert any(goal>1 for goal, _ in goals), details
        assert continuous or stopped, [d for d in statuses if d['status']!='READY']
        assert self.mission['phase']=='DONE' and samples[-1][0]>center[0]+.35, self.mission
        self.check('dynamic_replan_executed', mode='guarded_detour' if guarded else 'settled_detour' if stopped else 'native_continuous', **details)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=('routes','straight','outside','long','dynamic','blocked','static'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--contacts', action='store_true')
    parser.add_argument('--repetitions', type=int, default=5, choices=range(1, 6))
    args = parser.parse_args()
    args.mock = args.live = False
    args.yaw = 0.
    args.target = 'SITL planning fixture'
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11311':
        parser.error('expected local Gazebo SITL master')
    rospy.init_node('planning_sitl_validation', anonymous=True)
    if rospy.get_param('/use_sim_time',False) is not True:
        parser.error('Gazebo simulation clock required')
    rospy.wait_for_service('/gazebo/get_link_state', timeout=6.)
    trial = PlanningTrial(args)
    error = None
    contacts=None
    if args.contacts:
        from observe_physics_contacts import PhysicsContacts
        contacts=PhysicsContacts(args.out)
    try:
        if contacts:contacts.start()
        trial.configuration()
        assert trial.world(BODY, 'world').success
        if args.scenario in ('routes','straight','outside','long'):
            trial.route_matrix(args.repetitions,args.scenario)
        elif args.scenario=='static':
            if contacts:
                contacts.obstacle_model='segmented_test_block'
            trial.obstacle()
        else:
            trial.dynamic_box(blocked=args.scenario=='blocked')
    except Exception as exc:
        error = repr(exc)
        print('FAIL', error, flush=True)
    try:
        trial.stop_and_land()
        assert trial.state is not None and not trial.state.armed
    except Exception as exc:
        error = (error or '')+' CLEANUP '+repr(exc)
    if contacts:
        try:
            report=contacts.finish()
            assert report['passed'],report
            trial.check('physics_contact_stream_positive_control_and_no_iris_obstacle_contact',**report)
        except Exception as exc:error=(error or '')+' CONTACTS '+repr(exc)
    args.out.mkdir(parents=True, exist_ok=True)
    result = dict(passed=error is None, error=error, checks=trial.checks,
                  final_armed=trial.state.armed if trial.state else None)
    (args.out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    (args.out/'trace.json').write_text(json.dumps(trial.trace)+'\n')
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
