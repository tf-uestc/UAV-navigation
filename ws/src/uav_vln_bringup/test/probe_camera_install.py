#!/usr/bin/env python3
"""Read live camera/TF and run only the installed camera bridge. No control publishers."""
import json,os,signal,subprocess,time
from pathlib import Path
import message_filters,numpy as np,rospy,tf2_ros,yaml,rospkg
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped

out=Path('/root/autodl-tmp/uav/records/step78-2026-11-02')
rospy.init_node('camera_install_validation',anonymous=True,disable_signals=True)
depth_input=rospy.Subscriber('/iris_depth_camera/camera/depth/image_raw',Image,lambda _m:None,queue_size=1)
info=rospy.wait_for_message('/iris_depth_camera/camera/depth/camera_info',CameraInfo,timeout=10)
calibration=yaml.safe_load((Path(rospkg.RosPack().get_path('uav_vln_bringup'))/'config/ego_planner_params.yaml').read_text())['grid_map']
rospy.set_param('/camera_install_calibration',calibration)
rospy.set_param('/camera_pose_bridge_probe/grid_map_parameters','/camera_install_calibration')
f=(out/'camera-installed-node.log').open('w')
p=subprocess.Popen(['rosrun','uav_vln_bringup','camera_pose_bridge.py','__name:=camera_pose_bridge_probe'],
    stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
try:
    buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
    records=[]
    a=message_filters.Subscriber('/uav/planning/depth',Image)
    b=message_filters.Subscriber('/uav/planning/camera_pose',PoseStamped)
    sync=message_filters.TimeSynchronizer([a,b],30)
    def paired(image,pose):
        records.append((image,pose))
    sync.registerCallback(paired)
    limit=time.monotonic()+12
    while len(records)<15 and time.monotonic()<limit:time.sleep(.05)
    assert len(records)>=15,len(records)
    image,pose=records[-1]
    tf=buffer.lookup_transform('map','camera_link_optical',image.header.stamp,rospy.Duration(1))
    d=pose.pose.position;v=tf.transform.translation
    error=float(np.linalg.norm(np.array([d.x,d.y,d.z])-np.array([v.x,v.y,v.z])))
    assert image.header.frame_id=='camera_link_optical'
    assert pose.header.frame_id=='map'
    assert image.header.stamp==pose.header.stamp
    assert error<1e-9,error
    result=dict(passed=True,pairs=len(records),image_frame=image.header.frame_id,pose_frame=pose.header.frame_id,
       timestamp_equal=True,position_tf_error_m=error,encoding=image.encoding,width=image.width,height=image.height)

finally:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=3)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
    f.close()

# A mismatched calibration must close the map input, not silently distort it.
time.sleep(.3)
count=len(records)
rospy.set_param('/camera_install_calibration/fx',float(calibration['fx'])+1.)
rospy.set_param('/camera_pose_bridge_probe_bad/grid_map_parameters','/camera_install_calibration')
f=(out/'camera-bad-calibration.log').open('w')
p=subprocess.Popen(['rosrun','uav_vln_bringup','camera_pose_bridge.py','__name:=camera_pose_bridge_probe_bad'],
    stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
try:
    time.sleep(2)
    assert p.poll() is None
    assert len(records)==count, 'Mismatched calibration leaked map input'
    result['calibration_mismatch_rejected']=True
finally:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=3)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
    f.close()
    rospy.set_param('/camera_install_calibration/fx',float(calibration['fx']))
(out/'camera-installed-validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
