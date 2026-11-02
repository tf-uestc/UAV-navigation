#!/usr/bin/env python3
"""Supply one calibrated depth+optical-pose path to EGO; no cloud injection."""
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('camera_pose_bridge')
    buffer=tf2_ros.Buffer(rospy.Duration(30))
    listener=tf2_ros.TransformListener(buffer)
    info=None
    parameter_ns=rospy.get_param('~grid_map_parameters','/ego_planner_node/grid_map')
    fx,fy,cx,cy=[float(rospy.get_param(parameter_ns+'/'+key)) for key in ('fx','fy','cx','cy')]
    expected=np.array([fx,0,cx,0,fy,cy,0,0,1])
    depth_pub=rospy.Publisher('/uav/planning/depth',Image,queue_size=2)
    pose_pub=rospy.Publisher('/uav/planning/camera_pose',PoseStamped,queue_size=2)

    def on_info(m):
        nonlocal info
        if m.header.frame_id=='camera_link_optical' and np.allclose(m.K,expected,rtol=0,atol=1e-6):
            info=m
        else: info=None

    def on_depth(m):
        if info is None or m.encoding not in ('32FC1','16UC1') or m.header.frame_id!='camera_link_optical': return
        if m.width!=info.width or m.height!=info.height: return
        if abs((rospy.Time.now()-m.header.stamp).to_sec())>.3: return
        try:
            tf=buffer.lookup_transform('map',m.header.frame_id,m.header.stamp,rospy.Duration(.15))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException): return
        p=PoseStamped(); p.header.stamp=m.header.stamp
        p.header.frame_id='map'
        p.pose.position.x=tf.transform.translation.x
        p.pose.position.y=tf.transform.translation.y
        p.pose.position.z=tf.transform.translation.z
        p.pose.orientation=tf.transform.rotation
        pose_pub.publish(p)
        depth_pub.publish(m)

    rospy.Subscriber('/iris_depth_camera/camera/depth/camera_info',CameraInfo,on_info,queue_size=2)
    rospy.Subscriber('/iris_depth_camera/camera/depth/image_raw',Image,on_depth,queue_size=2)
    rospy.spin()

if __name__ == "__main__":
    main()
