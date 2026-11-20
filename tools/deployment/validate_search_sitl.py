#!/usr/bin/env python3
"""Validate autonomous red-pole search with isolated responses or live SITL."""
import argparse
import json
import math
import os
import threading
import time
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetLinkState, GetModelState, SetModelState
from std_msgs.msg import String
from tf.transformations import quaternion_matrix

from navigation_core.contracts import DetectionResult, RequestKey
from navigation_core.yaw import angle_delta
from validate_segments import Trial, decode

TARGET = '\u753b\u9762\u4e2d\u7ea2\u8272\u4e09\u811a\u652f\u67b6\u4e2d\u592e\u7684\u7ad6\u76f4\u7ea2\u6746\uff0c\u4ec5\u6846\u4f4f\u4e2d\u592e\u7ea2\u6746\u7684\u7ea2\u8272\u50cf\u7d20\uff0c\u5ffd\u7565\u5176\u4f59\u90e8\u5206\u548c\u5730\u9762\u3002\u627e\u5230\u65f6\u53ea\u8fd4\u56de\u4e00\u4e2a JSON \u5bf9\u8c61\uff0c\u4ec5\u7528 found\u3001bbox\u3001coordinate_system\uff0c\u5750\u6807\u5355\u4f4d\u5fc5\u987b\u662f normalized_1000\u3002\u770b\u4e0d\u5230\u4e2d\u592e\u7ea2\u6746\u5c31\u8fd4\u56de {"found":false}\u3002'
ORDER = [45,-45,90,-90]


def matrix(position, rotation):
    result = quaternion_matrix([rotation.x,rotation.y,rotation.z,rotation.w])
    result[:3,3] = [position.x,position.y,position.z]
    return result


class SearchTrial(Trial):
    def __init__(self, args):
        super().__init__(args)
        if args.live:
            self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(160.))
            self.listener = tf2_ros.TransformListener(self.tf)
            rospy.Subscriber('/uav/instruction', String, self.log_request, queue_size=10)

    def log_request(self, msg):
        data = json.loads(msg.data)
        if self.odom and self.flight and self.mission:
            _, yaw = decode(self.odom.pose.pose)
            self.record('search_capture', dict(
                request_id=data['request_id'], mission_id=data['mission_id'],
                floor=data['capture_after'], measured_yaw=yaw,
                scan_offset_deg=self.mission.get('scan_offset_deg',0),
                stop_confirmed=self.flight.get('stop_confirmed')))

    def detect_cb(self, msg):
        self.log_request(msg)
        data = json.loads(msg.data)
        scenario = self.scenarios.get(data['mission_id'])
        if not scenario:
            return
        scenario['calls'] += 1
        behavior = scenario['behavior']
        if behavior == 'silent':
            return
        offset = self.mission.get('scan_offset_deg', 0)
        reason = ''
        if behavior == 'invalid' or behavior == 'recover' and scenario['calls']==1:
            reason = 'invalid_bbox_response'
        elif behavior == 'timeout' and scenario['calls']==1:
            reason = 'api_timeout'
        elif behavior == 'missing' or behavior == 'last_heading' and offset != -90:
            reason = 'target_not_found'
        point = None if reason else scenario['target']
        if behavior=='wrong_first' and scenario['calls']==1:
            point=(point[0],point[1]+3.5,point[2])
        stamp = self.odom.header.stamp.to_sec()
        def respond():
            time.sleep(.10)
            result = DetectionResult(RequestKey(data['mission_id'],data['request_id']),
                                     self.args.target, 'FAILED' if reason else 'SUCCEEDED',
                                     stamp, rospy.Time.now().to_sec(), point, 'map', reason)
            self.detection.publish(String(data=result.to_json()))
        threading.Thread(target=respond, daemon=True).start()

    def mission_events(self, name, label):
        return [(t,d) for t,k,d in self.trace if k==label and d.get('mission_id')==name]

    def scans(self, name):
        return [d['scan_offset_deg'] for _,d in self.mission_events(name,'mission') if d['phase']=='SCAN_TURN']

    def deterministic(self):
        self.initial()
        name = self.new('invalid_twice', behavior='invalid', takeoff=True)
        result = self.wait_phase(name, ('FAILED','DONE'), 45)
        assert result['reason']=='invalid_bbox_response' and result['segment_count']==0,result
        captures = self.mission_events(name,'search_capture')
        assert len(captures)==2 and not self.scans(name), captures
        assert captures[1][1]['floor']>captures[0][1]['floor'], captures
        assert abs(angle_delta(captures[1][1]['measured_yaw'],captures[0][1]['measured_yaw']))<.05
        self.check('invalid_response_one_same_view_retry')
        name = self.new('not_found', behavior='missing')
        result = self.wait_phase(name, ('FAILED','DONE'), 55)
        assert result['reason']=='target_not_found' and result['segment_count']==0,result
        captures = [d for _,d in self.mission_events(name,'search_capture')]
        assert len(captures)==5 and self.scans(name)==ORDER, captures
        assert [d['scan_offset_deg'] for d in captures]==[0]+ORDER, captures
        assert all(d['stop_confirmed'] for d in captures),captures
        base = captures[0]['measured_yaw']
        for d in captures:
            error = abs(math.degrees(angle_delta(base+math.radians(d['scan_offset_deg']),d['measured_yaw'])))
            assert error<=7.,(d,error)
        assert not self.mission_events(name,'plan_request')
        self.check('full_scan_wrap_and_safe_no_target', headings=self.scans(name), requests=len(captures))
        name = self.new('conflicting_first_detection', behavior='wrong_first')
        result = self.wait_phase(name, ('FAILED','DONE'), 25)
        assert result['reason']=='initial_target_unconfirmed' and result['segment_count']==0,result
        assert not self.mission_events(name,'plan_request')
        assert len(self.mission_events(name,'search_capture'))==2
        self.check('inconsistent_initial_target_cannot_start_translation')
        name = self.new('recovered_after_timeout', behavior='timeout')
        result = self.wait_phase(name, ('FAILED','DONE'), 60)
        assert result['phase']=='DONE' and result['segment_count']>0,result
        assert not self.scans(name)
        self.check('cloud_timeout_retry_can_resume_with_new_valid_observation')

    def live_red_pole(self):
        self.initial()
        link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        red = model('red_tripod_A','world')
        assert red.success, red.status_message
        grounded=link('iris_depth_camera::iris::base_link','world')
        assert grounded.success
        base_truth=grounded.link_state.pose
        if self.args.bearing is not None:
            _, world_yaw=decode(base_truth)
            bearing=world_yaw+math.radians(self.args.bearing)
            expected=(base_truth.position.x+3.6*math.cos(bearing),
                      base_truth.position.y+3.6*math.sin(bearing))
            red.pose.position.x,red.pose.position.y=expected
            move=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
            moved=move(ModelState(model_name='red_tripod_A',pose=red.pose,reference_frame='world'))
            assert moved.success, moved.status_message
            time.sleep(.5)
            red=model('red_tripod_A','world')
            assert red.success and math.dist(expected,(red.pose.position.x,red.pose.position.y))<.02, red
        center = (red.pose.position.x,red.pose.position.y)
        stamp=self.odom.header.stamp
        base_tf=self.tf.lookup_transform('map','base_link',stamp,rospy.Duration(2.)).transform
        map_from_world=matrix(base_tf.translation,base_tf.rotation) @ np.linalg.inv(
            matrix(base_truth.position,base_truth.orientation))
        name = self.new('red_pole',takeoff=True)
        started = time.monotonic()
        minimum = math.inf
        camera_truth=[]
        end = None
        while time.monotonic()-started<self.args.mission_timeout+5. and not rospy.is_shutdown():
            world = link('iris_depth_camera::iris::base_link','world')
            assert world.success, world.status_message
            pos = world.link_state.pose.position
            distance = math.dist((pos.x,pos.y),center)
            minimum = min(minimum,distance)
            before=rospy.Time.now().to_sec()
            truth=link('iris_depth_camera::depth_camera::link','world')
            after=rospy.Time.now().to_sec()
            if truth.success and 0<=after-before<=.04:
                pose=truth.link_state.pose
                camera_truth.append(((before+after)/2.,matrix(pose.position,pose.orientation)))
            current = self.mission
            if current['mission_id']==name and current['phase'] in ('FAILED','DONE'):
                end = current
                break
            time.sleep(.065)
        assert end is not None, self.mission
        replies = [d for _,d in self.mission_events(name,'detect_result')]
        requests = self.mission_events(name,'search_capture')
        scans = self.scans(name)
        self.check('attempt_observations', mission_id=name, terminal=end['phase'], reason=end['reason'],
                   segments=end['segment_count'], scans=scans, detection_requests=len(requests),
                   responses=[dict(status=d['status'],reason=d['reason']) for d in replies],
                   distance_at_terminal_m=distance, minimum_horizontal_distance_m=minimum,
                   elapsed_seconds=time.monotonic()-started)
        assert end['phase']=='DONE' and end['reason']=='observation_standoff_reached',end
        assert self.args.expected==0 or self.args.expected in scans, scans
        assert scans==ORDER[:len(scans)], scans
        assert end['segment_count']>0
        localization=[]
        world_from_map=np.linalg.inv(map_from_world)
        for d in replies:
            if d['status']!='SUCCEEDED' or not camera_truth:
                continue
            captured=d['capture_stamp']
            nearest, truth=min(camera_truth,key=lambda item:abs(item[0]-captured))
            if abs(nearest-captured)>.075:
                continue
            capture_tf=self.tf.lookup_transform('map','camera_link_optical',
                                                rospy.Time.from_sec(captured),rospy.Duration(2.)).transform
            estimated=world_from_map @ matrix(capture_tf.translation,capture_tf.rotation)
            world_target=world_from_map @ np.array([*d['point'],1.])
            localization.append(dict(capture_stamp=captured,truth_time_delta_s=abs(nearest-captured),
                                     camera_translation_error_m=float(np.linalg.norm(estimated[:3,3]-truth[:3,3])),
                                     target_surface_error_m=abs(math.dist(world_target[:2],center)-.05)))
        assert localization, 'no time-matched historical TF and Gazebo camera observations'
        self.check('historical_tf_gazebo_and_target_surface', samples=localization)
        assert self.args.standoff-.35<=distance<=self.args.standoff+.35,(distance,self.args.standoff)
        reference_time=time.monotonic()
        time.sleep(10.)
        odom=[d for t,k,d in self.trace if t>=reference_time and k=='odom']
        assert odom and max(math.dist(d['position'],odom[0]['position']) for d in odom)<.30
        self.check('observed_and_held', distance_m=distance, hold_seconds=10.)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mock',action='store_true')
    parser.add_argument('--yaw',type=float)
    parser.add_argument('--expected',type=int,default=0)
    parser.add_argument('--bearing',type=float,
                        help='SITL fixture bearing relative to the initial camera heading')
    parser.add_argument('--standoff',type=float,default=2.4)
    parser.add_argument('--mission-timeout',type=float,default=720.)
    parser.add_argument('--out',required=True,type=Path)
    args=parser.parse_args()
    args.live=not args.mock
    args.target=TARGET
    port=11315 if args.mock else 11311
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:'+str(port):
        parser.error('expected isolated validation master')
    rospy.init_node('search_validation',anonymous=True)
    trial=SearchTrial(args)
    error=None
    try:
        if args.mock:
            trial.deterministic()
        else:
            trial.live_red_pole()
    except Exception as exc:
        error=repr(exc)
        print('FAIL',error,flush=True)
    try:
        trial.stop_and_land()
        assert trial.state and not trial.state.armed and trial.landed.landed_state==1
        trial.check('landed_disarmed')
    except Exception as exc:
        error=(error or '')+' CLEANUP '+repr(exc)
    finally:
        if trial.mock:
            trial.mock.done.set()
            trial.mock.worker.join(timeout=2.)
        args.out.mkdir(parents=True,exist_ok=True)
        (args.out/'results.json').write_text(json.dumps(dict(
            passed=error is None,error=error,checks=trial.checks,
            final_armed=trial.state.armed if trial.state else None),indent=2)+'\n')
        (args.out/'trace.json').write_text(json.dumps(trial.trace)+'\n')
    raise SystemExit(error is not None)


if __name__=='__main__':
    main()
