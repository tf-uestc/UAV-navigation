#!/usr/bin/env python3
"""Read-only comparison of historical map TF and the Gazebo target surface."""
import collections
import json
import math
from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetLinkState
import message_filters
from nav_msgs.msg import Odometry
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_matrix
import tf2_ros


def transform(translation, rotation):
    matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    matrix[:3,3] = translation.x, translation.y, translation.z
    return matrix


out = Path(sys.argv[1])
expected_heading = math.radians(float(sys.argv[2]))
candidate = None
ground = []


def yaw(matrix):
    return math.atan2(matrix[1,0], matrix[0,0])


def delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))
rospy.init_node('moving_rgbd_verification', anonymous=True)
buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.))
listener = tf2_ros.TransformListener(buffer)
bridge = CvBridge()
link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
queue = collections.deque(maxlen=2)
odom = collections.deque(maxlen=70)
recent = rospy.Subscriber('/mavros/local_position/odom', Odometry, odom.append, queue_size=5)
prefix = '/iris_depth_camera/camera/'
subs = [message_filters.Subscriber(prefix+suffix, kind, queue_size=2)
        for suffix,kind in (('rgb/image_raw', Image), ('depth/image_raw', Image), ('depth/camera_info', CameraInfo))]
sync = message_filters.TimeSynchronizer(subs, 4)
sync.registerCallback(lambda *images: queue.append(images))
optical_in_gazebo = np.eye(4)
optical_in_gazebo[:3,:3] = ((0,0,1),(-1,0,0),(0,-1,0))
observations = []
rejected = collections.Counter()
seen = 0
map_from_world = None
expires = time.monotonic()+110.
pending = collections.deque(maxlen=30)
while time.monotonic() < expires and len(observations) < 20 and not rospy.is_shutdown():
    time.sleep(.03)
    now = rospy.Time.now().to_sec()
    if pending and now-pending[0][0].to_sec() > .9:
        stamp, previous, sample = pending.popleft()
        again = buffer.lookup_transform('map','camera_link_optical',stamp,rospy.Duration(.15)).transform
        sample['delayed_tf_max_difference'] = float(np.max(np.abs(
            previous-transform(again.translation, again.rotation))))
        observations.append(sample)
    if not queue or not odom:
        continue
    rgb, depth, info = queue.popleft()
    stamp = rgb.header.stamp
    odom_msg = min(list(odom),key=lambda o: abs(o.header.stamp.to_sec()-stamp.to_sec()))
    v = odom_msg.twist.twist.linear
    speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
    if abs(odom_msg.header.stamp.to_sec()-stamp.to_sec()) > .04:
        continue
    try:
        if map_from_world is None:
            if speed > .05 or odom_msg.pose.pose.position.z > .4:
                continue
            truth = link('iris_depth_camera::iris::base_link','world')
            if not truth.success or abs(rospy.Time.now().to_sec()-stamp.to_sec()) > .08:
                continue
            base = buffer.lookup_transform('map','base_link',stamp,rospy.Duration(.12)).transform
            pose = truth.link_state.pose
            map_base = transform(base.translation,base.rotation)
            world_base = transform(pose.position,pose.orientation)
            alignment = map_base @ np.linalg.inv(world_base)
            entry = dict(stamp=stamp.to_sec(), map_yaw_deg=math.degrees(yaw(map_base)),
                         gazebo_yaw_deg=math.degrees(yaw(world_base)),
                         alignment_yaw_deg=math.degrees(yaw(alignment)),
                         alignment_translation_m=alignment[:3,3].tolist())
            ground.append(entry)
            if abs(delta(yaw(map_base),expected_heading)) > math.radians(3.):
                candidate = None
                continue
            if candidate is None:
                candidate = (stamp.to_sec(),alignment)
                continue
            if (abs(delta(yaw(candidate[1]),yaw(alignment))) > math.radians(.5)
                    or np.linalg.norm(candidate[1][:3,3]-alignment[:3,3]) > .035):
                candidate = (stamp.to_sec(),alignment)
                continue
            if stamp.to_sec()-candidate[0] >= 1.0:
                map_from_world = alignment
                out.with_name('alignment-ready.json').write_text(json.dumps(dict(
                    first=ground[0], accepted=entry, samples=ground),indent=2)+'\n')
            continue
        if math.hypot(v.x,v.y)<.07 or odom_msg.pose.pose.position.z<1.3:
            continue
        seen += 1
        pose_now = link('iris_depth_camera::depth_camera::link','world')
        lag = rospy.Time.now().to_sec()-stamp.to_sec()
        if not pose_now.success or not 0 <= lag <= .055:
            rejected['truth_time_window'] += 1
            continue
        known = pose_now.link_state.pose
        truth_camera = transform(known.position,known.orientation) @ optical_in_gazebo
        capture = buffer.lookup_transform('map','camera_link_optical',stamp,rospy.Duration(.12)).transform
        historical = transform(capture.translation,capture.rotation)
    except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
        if map_from_world is not None:
            rejected['tf_unavailable'] += 1
        continue
    color = bridge.imgmsg_to_cv2(rgb,'rgb8').astype(float)
    z = bridge.imgmsg_to_cv2(depth,'passthrough')
    r,g,b = np.moveaxis(color,-1,0)
    mask = (r>110)&(r>g*1.5)&(r>b*1.5)&np.isfinite(z)&(z>1)&(z<8)
    rows,cols = np.where(cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8)))
    if len(rows)<15:
        rejected['target_pixels'] += 1
        continue
    d=z[rows,cols]
    fx,fy,cx,cy=info.K[0],info.K[4],info.K[2],info.K[5]
    optical=np.stack(((cols-cx)*d/fx,(rows-cy)*d/fy,d,np.ones_like(d)))
    from_map=(np.linalg.inv(map_from_world)@historical@optical)[:3].T
    alignment_at_capture = historical @ np.linalg.inv(truth_camera)
    from_gazebo=(truth_camera@optical)[:3].T
    err=np.abs(np.hypot(from_map[:,0]-6.,from_map[:,1]-1.5)-.05)
    true_err=np.abs(np.hypot(from_gazebo[:,0]-6.,from_gazebo[:,1]-1.5)-.05)
    if np.percentile(true_err,95)>.045:
        rejected['ground_truth_projection'] += 1
        continue
    direction = truth_camera[:2,2]
    sample=dict(stamp=stamp.to_sec(),speed_mps=speed,truth_time_offset_s=lag,red_pixels=len(rows),
                heading_deg=math.degrees(math.atan2(direction[1], direction[0])),
                alignment_yaw_deg=math.degrees(yaw(alignment_at_capture)),
                yaw_alignment_delta_deg=math.degrees(delta(yaw(alignment_at_capture),yaw(map_from_world))),
                target_distance_m=float(np.linalg.norm(truth_camera[:2,3]-[6.,1.5])),
                map_surface_error_p95_m=float(np.percentile(err,95)),
                gazebo_surface_error_p95_m=float(np.percentile(true_err,95)))
    pending.append((stamp,historical,sample))
result=dict(samples=observations,count=len(observations),ground_alignment=map_from_world is not None,
            eligible_frames=seen,rejected=dict(rejected),pending=len(pending))
if observations:
    result['map_surface_error_p95_of_frame_p95_m']=float(np.percentile(
        [s['map_surface_error_p95_m'] for s in observations],95))
    result['history_exact']=all(s['delayed_tf_max_difference']<1e-8 for s in observations)
result['passed']=(result['count']>=10 and result.get('history_exact',False)
                  and result['map_surface_error_p95_of_frame_p95_m'] <= .20)
out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
raise SystemExit(0 if result['passed'] else 1)
