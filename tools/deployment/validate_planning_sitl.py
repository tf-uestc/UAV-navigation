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

from validate_segments import Trial, decode


BODY = 'iris_depth_camera::iris::base_link'


class PlanningTrial(Trial):
    def __init__(self, args):
        super().__init__(args)
        rospy.Subscriber('/uav/ego/status', GoalStatus, self.ego_status, queue_size=25)
        self.world = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    def ego_status(self, msg):
        if msg.mission_id:
            self.record('ego_status', dict(mission_id=msg.mission_id, goal_id=msg.goal_id,
                                          status=msg.status, reason=msg.reason))

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
        assert all(-6 <= d['position'][0] <= 6 and -4 <= d['position'][1] <= 4
                   and 1.1 <= d['position'][2] <= 2.35 for d in odom), 'measured fence breach'
        endpoint, _ = decode(self.odom.pose.pose)
        twist = self.odom.twist.twist.linear
        return dict(peak_tracking_error_m=max(errors),
                    rmse_tracking_m=math.sqrt(sum(x*x for x in errors)/len(errors)),
                    odom_samples=len(odom), aligned_samples=len(errors),
                    executed_trajectories=sorted({(d['context'][1], d['trajectory_id']) for d in tracking}),
                    endpoint=endpoint, endpoint_speed_mps=math.hypot(twist.x, twist.y, twist.z))

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

    def route_matrix(self, repetitions):
        self.takeoff()
        for i in range(repetitions):
            start, _ = decode(self.odom.pose.pose)
            delta = .9 if start[0] < 0.5 else -.9
            self.move('straight', (start[0] + delta, start[1], 1.6), 35)
            invalid = self.new('outside', waypoint=(6.5, 0., 1.6))
            result = self.wait_phase(invalid, ('DONE', 'FAILED'), 15)
            assert result['phase'] == 'FAILED' and result['reason'] == 'outside_fence', result
            assert not any(k == 'plan_request' and d['mission_id'] == invalid for _, k, d in self.trace)
            assert self.flight['mode'] == 'HOLD' and self.flight['context'] is None
            self.check('outside_fence', repetition=i + 1)
        self.move('position_for_long', (-4.2, -1.35, 1.6), 100)
        for i in range(repetitions):
            x = 4.2 if i % 2 == 0 else -4.2
            start, _ = decode(self.odom.pose.pose)
            goal = (x, -1.35, 1.6)
            distance = math.dist(start, goal)
            assert distance > 7.5
            result = self.move('beyond_planning_horizon', goal)
            assert result['segment_count'] >= 5

    def dynamic_box(self):
        self.takeoff()
        ground = self.world(BODY, 'world')
        assert ground.success
        p = ground.link_state.pose.position
        center = p.x + 1.14, p.y
        size = .16, .36, 2.6
        geometry = '<geometry><box><size>0.16 0.36 2.6</size></box></geometry>'
        sdf = ('<sdf version="1.6"><model name="dynamic_test_box"><static>true</static>'
               '<link name="body"><collision name="collision">'+geometry+'</collision>'
               '<visual name="visual">'+geometry+'</visual></link></model></sdf>')
        model = Pose(position=Point(*center, 1.3))
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
        while time.monotonic() - inserted < 50.:
            world = self.world(BODY, 'world')
            assert world.success
            point = world.link_state.pose.position
            samples.append((point.x, point.y, point.z))
            if self.mission['mission_id'] == mission and self.mission['phase'] in ('DONE', 'FAILED'):
                break
            time.sleep(.06)
        self.record('dynamic_box_world_track', dict(samples=samples))
        clearance = min(math.hypot(max(0., abs(q[0]-center[0])-.08),
                                   max(0., abs(q[1]-center[1])-.18)) for q in samples)
        self.check('dynamic_box_clearance', minimum_center_to_box_surface_m=clearance)
        assert clearance > .24, clearance
        statuses = [d for t, k, d in self.trace if t >= inserted and k == 'ego_status'
                    and d['mission_id'] == mission]
        assert any(d['status']=='REPLANNED' and d['reason']=='collision' for d in statuses), statuses
        details = self.track_metrics(mission, t0, time.monotonic())
        assert any(goal==1 and tid>first_id for goal, tid in details['executed_trajectories']), details
        assert self.mission['phase']=='DONE' and samples[-1][0]>center[0]+.35, self.mission
        self.check('dynamic_replan_executed', **details)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=('routes', 'dynamic'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--repetitions', type=int, default=5, choices=range(1, 6))
    args = parser.parse_args()
    args.mock = args.live = False
    args.yaw = 0.
    args.target = 'SITL planning fixture'
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11311':
        parser.error('expected local Gazebo SITL master')
    rospy.init_node('planning_sitl_validation', anonymous=True)
    rospy.wait_for_service('/gazebo/get_link_state', timeout=6.)
    trial = PlanningTrial(args)
    error = None
    try:
        assert trial.world(BODY, 'world').success
        if args.scenario == 'routes':
            trial.route_matrix(args.repetitions)
        else:
            trial.dynamic_box()
    except Exception as exc:
        error = repr(exc)
        print('FAIL', error, flush=True)
    try:
        trial.stop_and_land()
        assert trial.state is not None and not trial.state.armed
    except Exception as exc:
        error = (error or '')+' CLEANUP '+repr(exc)
    args.out.mkdir(parents=True, exist_ok=True)
    result = dict(passed=error is None, error=error, checks=trial.checks,
                  final_armed=trial.state.armed if trial.state else None)
    (args.out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    (args.out/'trace.json').write_text(json.dumps(trial.trace)+'\n')
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
