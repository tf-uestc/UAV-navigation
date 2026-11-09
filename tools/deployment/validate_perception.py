#!/usr/bin/env python3
"""Replay ROS perception faults or run two read-only requests on the Iris scene."""
import argparse
import json
import math
import os
import runpy
import subprocess
import sys
import time
import uuid
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import requests
import rospy
import tf2_ros
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from tf.transformations import quaternion_from_euler, quaternion_matrix


PREFIX = '/iris_depth_camera/camera/'
OPTICAL = 'camera_link_optical'


def fixture_node():
    from roslib.packages import find_node
    os.environ['VLM_API_KEY'] = 'offline-fixture'

    class Response:
        def __init__(self, text):
            self.text = text

        def raise_for_status(self):
            pass

        def json(self):
            return {'choices': [{'message': {'content': self.text}}]}

    def mock_post(url, headers, json, timeout):
        settings = rospy.get_param('/perception_fixture/reply')
        count = rospy.get_param('/perception_fixture/calls', 0)
        rospy.set_param('/perception_fixture/calls', count+1)
        assert headers['Authorization'] == 'Bearer offline-fixture'
        assert json['model'] == 'qwen3.8-omni-flash' and json['modalities'] == ['text']
        assert json['stream'] is False and json['response_format'] == {'type': 'json_object'}
        if settings.get('delay'):
            time.sleep(settings['delay'])
        if settings.get('failure') == 'network':
            raise requests.exceptions.ConnectionError('fixture')
        if settings.get('failure') == 'timeout':
            raise requests.exceptions.Timeout('fixture')
        return Response(settings['text'])

    with patch('qwen_vl_grounding.requests.post', side_effect=mock_post):
        runpy.run_path(find_node('uav_vln_bringup','detector_node.py')[0],run_name='__main__')


class Validation:
    def __init__(self):
        self.results = {}
        self.mission = 'perception-'+uuid.uuid4().hex
        self.request_id = 100
        self.checks = []
        self.pub = rospy.Publisher('/uav/instruction', String, queue_size=3)
        rospy.Subscriber('/uav/detection_result', String, self.receive, queue_size=10)

    def receive(self, msg):
        data = json.loads(msg.data)
        if data['mission_id'] == self.mission:
            self.results[data['request_id']] = data

    def wait(self, predicate, timeout):
        until = time.monotonic()+timeout
        while time.monotonic()<until and not rospy.is_shutdown():
            value = predicate()
            if value:
                return value
            time.sleep(.02)
        raise AssertionError('timed out waiting for expected ROS feedback')

    def query(self, text):
        self.wait(self.pub.get_num_connections,10)
        self.request_id += 7
        wall = time.monotonic()
        self.pub.publish(String(data=json.dumps(dict(target={'text':text},
            _mission_mgr_trigger=True,mission_id=self.mission,request_id=self.request_id))))
        result = self.wait(lambda: self.results.get(self.request_id),18)
        assert result['target_text']==text,result
        return result, round(time.monotonic()-wall,3)

    def check(self, name, **data):
        self.checks.append(dict(name=name, **data))
        print('PASS',name,data,flush=True)

    def fixtures(self, sample_dir, captured_response):
        bridge = CvBridge()
        rgb_pub = rospy.Publisher(PREFIX+'rgb/image_raw',Image,queue_size=2)
        depth_pub = rospy.Publisher(PREFIX+'depth/image_raw',Image,queue_size=2)
        info_pub = rospy.Publisher(PREFIX+'depth/camera_info',CameraInfo,queue_size=2)
        tf = tf2_ros.TransformBroadcaster()
        self.wait(lambda: all(p.get_num_connections() for p in (rgb_pub,depth_pub,info_pub)),8)
        fake = dict(found=True,bbox=[125,125,875,875],coordinate_system='normalized_1000')
        sample = np.zeros((24,32,3),dtype=np.uint8)
        depth = np.full((24,32),10.,dtype=np.float32)
        depth[8:16,10:18] = 3.
        depth[5,5] = .2
        K = [100.,0.,16.,0.,100.,12.,0.,0.,1.]

        def send_frame(bgr, values, calibration, position, yaw, stamp=None, skew=0., frame=OPTICAL,
                       info_frame=OPTICAL, publish_tf=True, encoding='32FC1'):
            stamp = stamp or rospy.Time.now()
            for _ in range(3):
                if publish_tf:
                    tr = TransformStamped(header=Header(stamp=stamp,frame_id='map'),child_frame_id=OPTICAL)
                    tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z = position
                    q = quaternion_from_euler(0.,0.,yaw)
                    tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w = q
                    tf.sendTransform(tr)
                rgb = bridge.cv2_to_imgmsg(bgr,'bgr8')
                rgb.header = Header(stamp=stamp,frame_id=frame)
                dm = bridge.cv2_to_imgmsg(values,encoding)
                dm.header = Header(stamp=stamp+rospy.Duration(skew),frame_id=frame)
                ci = CameraInfo(header=Header(stamp=stamp,frame_id=info_frame),
                                width=values.shape[1],height=values.shape[0],K=calibration)
                rgb_pub.publish(rgb)
                depth_pub.publish(dm)
                info_pub.publish(ci)
                time.sleep(.03)
            return stamp.to_sec()

        def fixture(reply=fake, **options):
            rospy.set_param('/perception_fixture/reply',dict(text=json.dumps(reply),**options))

        def positive(label, values, position, yaw, true_depth):
            fixture()
            encoding = '16UC1' if values.dtype == np.uint16 else '32FC1'
            stamp = send_frame(sample,values,K,position,yaw,encoding=encoding)
            result,_ = self.query(label)
            point = np.array(result['point'])
            dx,dy = (13.5-16)*true_depth/100,(11.5-12)*true_depth/100
            expected = np.array(position)+[math.cos(yaw)*dx-math.sin(yaw)*dy,
                                           math.sin(yaw)*dx+math.cos(yaw)*dy,true_depth]
            assert result['status']=='SUCCEEDED' and abs(result['capture_stamp']-stamp)<1e-6,result
            error = float(np.linalg.norm(point-expected))
            assert error<1e-5,(point,expected)
            self.check(label,position_error_m=error)

        positive('foreground_over_background_and_near_outlier',depth,(1.,2.,0.),0.,3.)
        deeper = depth.copy()
        deeper[8:16,10:18] = 5.
        positive('different_pose_range_and_millimeter_units',(deeper*1000).astype(np.uint16),
                 (-2.,3.,.5),math.pi/2,5.)
        baseline = json.loads((sample_dir/'validation.json').read_text())
        fixture(json.loads(captured_response.read_text()))
        original = cv2.imread(str(sample_dir/'rgb.png'))
        original_depth = np.load(sample_dir/'depth-meters.npy')
        send_frame(original,original_depth,baseline['image']['K'],(0.,0.,0.),0.)
        result,_ = self.query('captured_scene_with_recorded_cloud_reply')
        assert result['status']=='SUCCEEDED',result
        depth_error = abs(result['point'][2]-baseline['red_target']['depth_median_m'])
        assert depth_error < .05,depth_error
        self.check('captured_image_response_replay',depth_error_m=depth_error)

        for name, settings, values, expected in (
            ('no_target',{'reply':{'found':False}},depth,'target_not_found'),
            ('wrong_units',{'reply':dict(fake,coordinate_system='pixel')},depth,'invalid_bbox_response'),
            ('invalid_depth',{},np.full_like(depth,np.nan),'invalid_target_depth'),
            ('network_failure',{'failure':'network'},depth,'api_network_error'),
            ('timeout',{'failure':'timeout'},depth,'api_timeout'),
            ('expired_reply',{'delay':2.2},depth,'detection_expired'),
        ):
            fixture(**settings)
            send_frame(sample,values,K,(0.,0.,0.),0.)
            result,_ = self.query(name)
            assert result['status']=='FAILED' and result['reason']==expected and result['point'] is None,result
            self.check(name,reason=expected)

        fixture(delay=.6)
        stamp = send_frame(sample,depth,K,(1.,2.,0.),0.)
        calls = rospy.get_param('/perception_fixture/calls')
        self.request_id += 7
        self.pub.publish(String(data=json.dumps(dict(target={'text':'moving_tf'},_mission_mgr_trigger=True,
            mission_id=self.mission,request_id=self.request_id))))
        self.wait(lambda: rospy.get_param('/perception_fixture/calls')>calls,2)
        send_frame(sample,depth,K,(11.,22.,0.),0.)
        result = self.wait(lambda: self.results.get(self.request_id),4)
        assert result['status']=='SUCCEEDED' and abs(result['capture_stamp']-stamp)<1e-6,result
        assert np.linalg.norm(np.array(result['point'])-np.array([.925,1.985,3.]))<1e-5,result
        self.check('delayed_detection_uses_captured_tf')

        fixture()
        calls = rospy.get_param('/perception_fixture/calls')
        send_frame(sample,depth,K,(0.,0.,0.),0.,skew=.09)
        time.sleep(1.05)
        result,_ = self.query('unpaired_depth')
        assert result['status']=='FAILED' and result['point'] is None,result
        assert rospy.get_param('/perception_fixture/calls')==calls
        self.check('unpaired_depth_never_calls_api',reason=result['reason'])
        send_frame(sample,depth,K,(0.,0.,0.),0.,info_frame='wrong_frame')
        result,_ = self.query('wrong_camera_info')
        assert result['status']=='FAILED' and result['reason']=='camera_info_frame_mismatch',result
        self.check('camera_info_frame_mismatch')
        send_frame(sample,depth,K,(0.,0.,0.),0.,publish_tf=False)
        result,_ = self.query('missing_tf')
        assert result['reason']=='historical_tf_unavailable' and result['point'] is None,result
        assert rospy.get_param('/perception_fixture/calls')==calls
        self.check('missing_historical_tf_never_calls_api')

    def live(self):
        state = rospy.wait_for_message('/mavros/state',State,timeout=5)
        assert state.connected and not state.armed
        tf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf)
        self.wait(lambda: tf.can_transform('map','base_link',rospy.Time(0)),6)
        rospy.wait_for_message(PREFIX+'rgb/image_raw',Image,timeout=5)
        time.sleep(.3)
        result,latency = self.query('图中的红色三脚支架（红色三脚杆），标出整个支架')
        assert result['status']=='SUCCEEDED',result
        stamp = rospy.Time.from_sec(result['capture_stamp'])
        tr = tf.lookup_transform('map','base_link',stamp,rospy.Duration(2.)).transform
        link = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)('iris_depth_camera::iris::base_link','world')
        assert link.success
        pose = link.link_state.pose

        def matrix(q,p):
            transform = quaternion_matrix((q.x,q.y,q.z,q.w))
            transform[:3,3] = [p.x,p.y,p.z]
            return transform

        world_from_map = matrix(pose.orientation,pose.position)@np.linalg.inv(matrix(tr.rotation,tr.translation))
        world = (world_from_map@np.array([*result['point'],1.]))[:3]
        surface_error = abs(math.hypot(world[0]-6.,world[1]-1.5)-.05)
        assert surface_error < .20 and 0. < world[2] < 1.3,(world,surface_error)
        self.check('live_red_target_world_localization',latency_s=latency,world_xyz_m=world.tolist(),
                   horizontal_cylinder_surface_error_m=surface_error)
        result,latency = self.query('亮紫色双层公交车。只有能明确看到公交车时才标注')
        assert result['status']=='FAILED' and result['reason']=='target_not_found' and result['point'] is None,result
        self.check('live_absent_target_has_no_world_point',latency_s=latency)
        state = rospy.wait_for_message('/mavros/state',State,timeout=5)
        assert state.connected and not state.armed
        assert listener is not None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fixture',action='store_true')
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--sample',type=Path)
    parser.add_argument('--response',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args = parser.parse_args()
    if args.fixture == args.live:
        parser.error('select exactly one mode')
    port = 11316 if args.fixture else 11311
    if os.environ.get('ROS_MASTER_URI') != f'http://127.0.0.1:{port}':
        parser.error('wrong ROS master for requested test mode')
    rospy.init_node('perception_validation',anonymous=True)
    validation = Validation()
    node = None
    error = None
    try:
        if args.fixture:
            node = subprocess.Popen([sys.executable,__file__,'--fixture-node','_synchronize_camera:=true',
                                     '_managed_requests_only:=true','_max_result_age:=2.'])
            validation.fixtures(args.sample,args.response)
        else:
            validation.live()
    except Exception as exc:
        error = repr(exc)
        print('FAIL',error,flush=True)
    finally:
        if node:
            node.terminate()
            node.wait(timeout=6)
        args.out.write_text(json.dumps(dict(passed=error is None, checks=validation.checks,
                                           error=error),indent=2)+'\n')
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    if '--fixture-node' in sys.argv:
        fixture_node()
    else:
        main()
