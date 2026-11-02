#!/usr/bin/env python3
"""Read-only static RGB-D/TF validation against Gazebo scene truth; never arms."""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetLinkState, GetPhysicsProperties, GetWorldProperties
from mavros_msgs.msg import State
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf.transformations import quaternion_matrix, quaternion_from_matrix

parser = argparse.ArgumentParser()
parser.add_argument('--phase', default='after')
parser.add_argument('--output-dir', default='/root/autodl-tmp/uav/records/step56-2026-11-02')
args = parser.parse_args()
out = Path(args.output_dir) / args.phase
out.mkdir(parents=True, exist_ok=True)
rospy.init_node('validate_project_rgbd', anonymous=True, disable_signals=True)
bridge = CvBridge()
buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
listener = tf2_ros.TransformListener(buffer)
prefix = '/iris_depth_camera/camera/'
counts = {}
latest = {}
times = {}
subscribers = []
sampling = {'active':False}
first_age = {}

def observe(m, key):
    first_age.setdefault(key, rospy.Time.now().to_sec()-m.header.stamp.to_sec())
    if not sampling['active']:
        return
    counts[key] = counts.get(key, 0) + 1
    latest[key] = m
    times.setdefault(key, []).append((time.monotonic(), m.header.stamp.to_sec()))

for key, topic, typ in [('rgb','rgb/image_raw',Image), ('depth','depth/image_raw',Image),
                        ('rgb_info','rgb/camera_info',CameraInfo),
                        ('depth_info','depth/camera_info',CameraInfo),
                        ('cloud','depth/points',PointCloud2)]:
    subscribers.append(rospy.Subscriber(prefix + topic, typ, observe, key, queue_size=2))
subscribers.append(rospy.Subscriber('/mavros/state', State, observe, 'state', queue_size=5))
sync_inputs = [message_filters.Subscriber(prefix+t, typ, queue_size=3) for t,typ in
               [('rgb/image_raw',Image), ('depth/image_raw',Image),
                ('rgb/camera_info',CameraInfo), ('depth/camera_info',CameraInfo),
                ('depth/points',PointCloud2)]]
sync = message_filters.TimeSynchronizer(sync_inputs, 30)
sync_result = {'count': 0}

def paired(*messages):
    if not sampling['active']:
        return
    sync_result['count'] += 1
    sync_result['messages'] = messages

sync.registerCallback(paired)
time.sleep(2)  # Exclude the plugin's initially latched cached image from steady-state rates.
sampling['active'] = True
deadline = time.monotonic() + 10
while time.monotonic() < deadline:
    time.sleep(.05)
result = {'phase':args.phase, 'topics':{}, 'checks':{}, 'exact_sync_count':sync_result['count'],
          'first_message_age_s':first_age, 'warmup_wall_s':2}
checks = result['checks']
for key, n in counts.items():
    ts = times[key]
    result['topics'][key] = {'count':n, 'frame':latest[key].header.frame_id,
                            'stamp_advance':ts[-1][1]-ts[0][1]}
    if len(ts)>1:
        result['topics'][key]['wall_rate_hz']=(len(ts)-1)/(ts[-1][0]-ts[0][0])
        result['topics'][key]['stamps_monotonic']=all(b[1]>=a[1] for a,b in zip(ts,ts[1:]))
checks['all_topics_received'] = all(counts.get(k,0)>5 for k in ['rgb','depth','rgb_info','depth_info','cloud','state'])
checks['exact_rgb_depth_info_cloud_sync'] = sync_result['count']>5
checks['steady_state_timestamps_advance'] = all(result['topics'].get(k,{}).get('stamp_advance',0)>5 and result['topics'][k].get('stamps_monotonic',False) for k in ['rgb','depth','rgb_info','depth_info','cloud'])
state = latest.get('state')
result['state'] = {'connected':bool(state and state.connected), 'armed':bool(state and state.armed)}
checks['connected_unarmed'] = bool(state and state.connected and not state.armed)
world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
result['world_models'] = world.model_names
checks['custom_scene'] = all(n in world.model_names for n in ['iris_depth_camera','red_tripod_A','blue_tripod_B','landing_pad'])
physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)()
result['gravity'] = [physics.gravity.x,physics.gravity.y,physics.gravity.z]
checks['configured_gravity'] = abs(physics.gravity.z+9.8066)<1e-6

def matrix(pose):
    q = pose.orientation
    t = quaternion_matrix([q.x,q.y,q.z,q.w])
    t[:3,3] = [pose.position.x,pose.position.y,pose.position.z]
    return t

def tf_matrix(t):
    q=t.transform.rotation
    m=quaternion_matrix([q.x,q.y,q.z,q.w])
    v=t.transform.translation
    m[:3,3]=[v.x,v.y,v.z]
    return m

if sync_result.get('messages'):
    rgb, depth, rgb_info, depth_info, cloud = sync_result['messages']
    color = bridge.imgmsg_to_cv2(rgb,'rgb8')
    z = bridge.imgmsg_to_cv2(depth,'passthrough')
    cv2.imwrite(str(out/'rgb.png'),cv2.cvtColor(color,cv2.COLOR_RGB2BGR))
    np.save(str(out/'depth-meters.npy'),z)
    result['image']={'rgb_encoding':rgb.encoding,'depth_encoding':depth.encoding,
                     'width':rgb.width,'height':rgb.height,'K':list(depth_info.K),
                     'valid_depth_fraction':float(np.isfinite(z).mean()),
                     'frames':[m.header.frame_id for m in (rgb,depth,rgb_info,depth_info,cloud)]}
    checks['optical_frame_headers'] = all(m.header.frame_id=='camera_link_optical' for m in (rgb,depth,rgb_info,depth_info,cloud))
    checks['aligned_resolution_intrinsics'] = bool(rgb.width==depth.width==640 and rgb.height==depth.height==480 and np.allclose(rgb_info.K,depth_info.K))
    checks['float_depth_meters'] = depth.encoding=='32FC1' and z.dtype==np.float32
    k=depth_info.K
    fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[0,4,8],'itemsize':cloud.point_step})
    points=np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=cloud.data,strides=(cloud.row_step,cloud.point_step))
    v,u=np.mgrid[0:depth.height:20,0:depth.width:20]
    zs=z[v,u].astype(float)
    expected=np.stack(((u-cx)*zs/fx,(v-cy)*zs/fy,zs),axis=-1)
    actual=np.stack([points[c][v,u] for c in ('x','y','z')],axis=-1)
    valid=np.isfinite(expected).all(axis=-1)&np.isfinite(actual).all(axis=-1)
    cloud_error=float(np.max(np.linalg.norm(actual[valid]-expected[valid],axis=-1)))
    result['organized_cloud_projection_max_error_m']=cloud_error
    checks['cloud_matches_optical_backprojection'] = cloud_error<1e-3
    link = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
    base_world=link('iris_depth_camera::iris::base_link','world')
    cam_world=link('iris_depth_camera::depth_camera::link','world')
    cam_base=link('iris_depth_camera::depth_camera::link','iris_depth_camera::iris::base_link')
    if not all(x.success for x in (base_world,cam_world,cam_base)):
        raise RuntimeError('Gazebo link truth unavailable')
    result['camera_mount_gazebo_m']=list(matrix(cam_base.link_state.pose)[:3,3])
    stamp=rgb.header.stamp
    map_optical=tf_matrix(buffer.lookup_transform('map','camera_link_optical',stamp,rospy.Duration(3)))
    map_base=tf_matrix(buffer.lookup_transform('map','base_link',stamp,rospy.Duration(3)))
    base_optical=tf_matrix(buffer.lookup_transform('base_link','camera_link_optical',stamp,rospy.Duration(3)))
    # Gazebo camera link is FLU; image/point cloud coordinates are RDF optical.
    flu_from_optical=np.eye(4)
    flu_from_optical[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    expected_base_optical=matrix(cam_base.link_state.pose)@flu_from_optical
    offset_error=float(np.linalg.norm(base_optical[:3,3]-expected_base_optical[:3,3]))
    rotation_error=float(math.acos(np.clip((np.trace(base_optical[:3,:3].T@expected_base_optical[:3,:3])-1)/2,-1,1)))
    result['mount_translation_error_m']=offset_error
    result['optical_rotation_error_deg']=math.degrees(rotation_error)
    result['base_from_optical_tf']=base_optical.tolist()
    checks['camera_mount_matches_gazebo'] = offset_error<.002 and rotation_error<.005
    checks['historical_map_tf_available'] = True
    # Static pose alignment is measured here; map origin is not assumed equal to world.
    map_from_world=map_base@np.linalg.inv(matrix(base_world.link_state.pose))
    world_from_optical_tf=np.linalg.inv(map_from_world)@map_optical
    world_from_optical_truth=matrix(cam_world.link_state.pose)@flu_from_optical
    r,g,b=[color[:,:,i].astype(float) for i in range(3)]
    depth_for_mask=np.nan_to_num(z,nan=0,posinf=0,neginf=0)
    mask=(r>110)&(r>g*1.5)&(r>b*1.5)&(depth_for_mask>1)&(depth_for_mask<10)
    mask=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
    vv,uu=np.where(mask)
    result['red_target_pixels']=len(vv)
    checks['red_target_visible']=len(vv)>20
    if len(vv):
        zz=z[vv,uu]
        optical=np.stack([(uu-cx)*zz/fx,(vv-cy)*zz/fy,zz,np.ones_like(zz)])
        reconstructed=(world_from_optical_tf@optical)[:3].T
        independent=(world_from_optical_truth@optical)[:3].T
        radial=np.sqrt((reconstructed[:,0]-6)**2+(reconstructed[:,1]-1.5)**2)
        radial_errors=np.abs(radial-.05)
        result['red_target']={'depth_median_m':float(np.median(zz)),
           'world_surface_median':np.median(reconstructed,axis=0).tolist(),
           'cylinder_surface_error_median_m':float(np.median(radial_errors)),
           'cylinder_surface_error_p95_m':float(np.percentile(radial_errors,95)),
           'tf_vs_gazebo_projection_max_error_m':float(np.max(np.linalg.norm(reconstructed-independent,axis=1)))}
        checks['red_surface_matches_world'] = bool(np.percentile(radial_errors,95)<.03 and np.median(zz)>4 and np.median(zz)<6)
        checks['map_camera_chain_matches_truth'] = result['red_target']['tf_vs_gazebo_projection_max_error_m']<.01
    # Visual QA only: depth uses metres; this image is clipped to 10 m.
    vis=np.nan_to_num(z,nan=0,posinf=0,neginf=0)
    vis=cv2.applyColorMap((np.clip(vis/10,0,1)*255).astype(np.uint8),cv2.COLORMAP_TURBO)
    cv2.imwrite(str(out/'depth-preview.png'),vis)
else:
    checks['synchronized_images_available']=False
result['pass']=all(checks.values())
result['scope']='Static ground validation only; no arming, movement, API calls or navigation integration.'
(out/'validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
raise SystemExit(0 if result['pass'] else 1)
