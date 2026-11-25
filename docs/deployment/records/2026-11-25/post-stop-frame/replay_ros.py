"""Reproduce the post-stop frame race through actual ROS and TimeSynchronizer."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String


def wait(fn, seconds=5.):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        value = fn()
        if value:
            return value
        time.sleep(.005)
    raise RuntimeError('timed out waiting for replay transport or evidence')


def run(source):
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11327':
        raise RuntimeError('Dedicated replay master required')
    spec = importlib.util.spec_from_file_location('under_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.DetectorNode()
    assert isinstance(node.detector.calls, list) and node._synchronize_camera
    assert rospy.get_param('/use_sim_time') is True
    bridge = CvBridge()
    clock = rospy.Publisher('/clock', Clock, queue_size=2, latch=True)
    cmd = rospy.Publisher('/uav/instruction', String, queue_size=5)
    rgb_pub = rospy.Publisher('/iris_depth_camera/camera/rgb/image_raw', Image, queue_size=3)
    depth_pub = rospy.Publisher('/iris_depth_camera/camera/depth/image_raw', Image, queue_size=3)
    info_pub = rospy.Publisher('/iris_depth_camera/camera/depth/camera_info', CameraInfo, queue_size=3)
    replies, states = [], []
    subscribers = [rospy.Subscriber(topic, String, lambda m, rows: rows.append(
        (time.monotonic(), json.loads(m.data))), dest, queue_size=20) for topic,dest in
        [('/uav/detection_result',replies),('/uav/detector_status',states)]]
    wait(lambda: all(pub.get_num_connections() for pub in (cmd,rgb_pub,depth_pub,info_pub,clock))
         and all(s.get_num_connections() for s in subscribers))

    def sim(value):
        clock.publish(Clock(clock=rospy.Time.from_sec(value)))
        wait(lambda: abs(rospy.Time.now().to_sec()-value) < 1.e-6)

    def frame(stamp, parts=('rgb','depth','info')):
        header=Header(stamp=rospy.Time.from_sec(stamp),frame_id=node.camera_optical_frame)
        if 'rgb' in parts:
            msg=bridge.cv2_to_imgmsg(np.zeros((4,4,3),dtype=np.uint8),'bgr8')
            msg.header=header
            rgb_pub.publish(msg)
        if 'depth' in parts:
            msg=bridge.cv2_to_imgmsg(np.full((4,4),2000,dtype=np.uint16),'16UC1')
            msg.header=header
            depth_pub.publish(msg)
        if 'info' in parts:
            info_pub.publish(CameraInfo(header=header,width=4,height=4,K=[2.,0.,2.,0.,2.,2.,0.,0.,1.]))

    def request(rid,floor,mission='replay'):
        cmd.publish(String(data=json.dumps(dict(mission_id=mission, request_id=rid,
                    _mission_mgr_trigger=True,target={'text':'fixture'},capture_after=floor))))

    def result(rid):
        return wait(lambda:next(((t,d) for t,d in replies if d['request_id']==rid),None))

    def start(rid):
        return wait(lambda:next((t for t,d in states if d['request_id']==rid and d['status']=='STARTED'),None))

    sim(51.436)
    tf = tf2_ros.StaticTransformBroadcaster()
    tr=TransformStamped(header=Header(stamp=rospy.Time.now(),frame_id='map'),child_frame_id=node.camera_optical_frame)
    tr.transform.rotation.w=1.
    tf.sendTransform(tr)
    wait(lambda:node.tf_buffer.can_transform('map',node.camera_optical_frame,rospy.Time.from_sec(51.316)))
    frame(51.22)
    wait(lambda:node.latest_rgb is not None and abs(node.latest_rgb.header.stamp.to_sec()-51.22)<1.e-6)
    request(7,51.236)
    start(7)
    frame(51.316,('rgb',))
    time.sleep(.1)
    partial_had_no_call = not node.detector.calls
    frame(51.316,('depth','info'))
    _,r7=result(7)
    wait(lambda:abs(node.latest_rgb.header.stamp.to_sec()-51.316)<1.e-6 and not node._detecting)

    request(8,51.4)
    t8=start(8)
    request(9,51.4,'different-mission')
    _,r9=result(9)
    t,r8=result(8)
    elapsed=t-t8
    wait(lambda:not node._detecting)
    pre=len(node.detector.calls)
    frame(51.416)
    wait(lambda:abs(node.latest_rgb.header.stamp.to_sec()-51.416)<1.e-6)
    time.sleep(.05)
    no_revive=(len(node.detector.calls)==pre and len([d for _,d in replies if d['request_id']==8])==1)
    request(10,51.4)
    _,r10=result(10)
    wait(lambda:not node._detecting)
    sim(53.436)
    request(11,51.41)
    _,r11=result(11)
    checks = dict(subframe_without_depth_info_never_calls_backend=partial_had_no_call,
                  first_qualified_triplet_succeeds=r7['status']=='SUCCEEDED' and r7['mission_id']=='replay'
                    and r7['point']==[0.,0.,2.] and abs(r7['capture_stamp']-51.316)<1.e-6,
                  wall_timeout_while_sim_clock_paused=r8['reason']=='post_stop_frame_timeout'
                    and r8['status']=='FAILED' and r8['capture_stamp']==0. and .8<=elapsed<=2.,
                  concurrent_request_retains_own_key_and_returns_busy=r9['status']=='BUSY'
                    and r9['mission_id']=='different-mission',
                  late_frame_cannot_revive_closed_request=no_revive,
                  next_request_captures_fresh_frame=r10['status']=='SUCCEEDED' and abs(r10['capture_stamp']-51.416)<1.e-6,
                  stale_eligible_frame_rejected=r11['reason']=='stale image' and r11['point'] is None,
                  only_two_model_calls=len(node.detector.calls)==2)
    return dict(passed=all(checks.values()), checks=checks, timeout_wall_s=elapsed, backend_calls=len(node.detector.calls),
                responses=[r7,r8,r9,r10,r11],python=sys.version.split()[0],master=os.environ['ROS_MASTER_URI'])


if __name__=='__main__':
    report=run(Path(sys.argv[1]))
    Path(sys.argv[2]).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['checks']))
    raise SystemExit(not report['passed'])
