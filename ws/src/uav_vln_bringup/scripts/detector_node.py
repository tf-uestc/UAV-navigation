#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detector_node.py — ROS detector node for UAV visual grounding.

Subscribes to image topics and /uav/instruction, loads a Detector
implementation via importlib, runs detection in a background thread,
transforms results to map frame via TF, and publishes target points
and debug images.

Topics:
  Subscribe:
    ~camera_topic_rgb    (sensor_msgs/Image)      - RGB camera
    ~camera_topic_depth  (sensor_msgs/Image)      - Depth camera
    ~camera_topic_info   (sensor_msgs/CameraInfo) - Camera intrinsics
    /uav/instruction     (std_msgs/String)         - JSON target spec

  Publish:
    /uav/target_world    (geometry_msgs/PointStamped) - 3D target in map
    /uav/target_debug    (sensor_msgs/Image)          - Annotated debug view

Params:
  ~detector_class       (str)   - importlib path to Detector subclass
  ~detector_args        (dict)  - kwargs forwarded to detector constructor
  ~vlm_api_key          (str)   - API key (falls back to VLM_API_KEY env)
  ~target_frame         (str)   - TF target frame (default "map")
  ~camera_optical_frame (str)   - Optical frame ID (default "camera_link_optical")
  ~trigger_on_start     (bool)  - Reserved for future use
"""
import importlib
import json
import os
import threading
import time

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String

from base_detector import Detection, Detector


class DetectorNode:
    """ROS node wrapping a Detector implementation with TF and debug viz."""

    def __init__(self):
        rospy.init_node("detector_node")

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # --- Latest sensor data (updated by subscribers) ---
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_cam_info = None

        # --- API key resolution ---
        vlm_api_key = rospy.get_param("~vlm_api_key", "")
        if not vlm_api_key:
            vlm_api_key = os.environ.get("VLM_API_KEY", "")

        # --- Load detector via importlib ---
        cls_path = rospy.get_param("~detector_class", "dummy_detector.DummyDetector")
        detector_args = rospy.get_param("~detector_args", {})
        if vlm_api_key and "api_key" not in detector_args:
            detector_args["api_key"] = vlm_api_key
        self.detector = self._load_detector(cls_path, detector_args)
        rospy.loginfo("[detector] Loaded detector: %s", cls_path)

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.camera_optical_frame = rospy.get_param(
            "~camera_optical_frame", "camera_link_optical"
        )

        # --- Reserved future feature ---
        trigger_on_start = rospy.get_param("~trigger_on_start", False)
        if trigger_on_start:
            rospy.loginfo("[detector] trigger_on_start=true (future feature)")

        # --- Subscribers ---
        camera_rgb = rospy.get_param(
            "~camera_topic_rgb", "/iris_depth_camera/camera/rgb/image_raw"
        )
        camera_depth = rospy.get_param(
            "~camera_topic_depth", "/iris_depth_camera/camera/depth/image_raw"
        )
        camera_info = rospy.get_param(
            "~camera_topic_info", "/iris_depth_camera/camera/depth/camera_info"
        )
        rospy.Subscriber(camera_rgb, Image, self._cb_rgb, queue_size=1)
        rospy.Subscriber(camera_depth, Image, self._cb_depth, queue_size=1)
        rospy.Subscriber(camera_info, CameraInfo, self._cb_cam_info, queue_size=1)
        rospy.Subscriber("/uav/instruction", String, self._cb_instruction, queue_size=1)

        # --- Publishers ---
        self.pub_target = rospy.Publisher(
            "/uav/target_world", PointStamped, queue_size=1
        )
        self.pub_debug = rospy.Publisher("/uav/target_debug", Image, queue_size=1)

        rospy.loginfo("[detector] DetectorNode initialized successfully")

    # ------------------------------------------------------------------
    # Importlib loader
    # ------------------------------------------------------------------
    def _load_detector(self, cls_path, kwargs):
        """Instantiate a Detector subclass from a dotted import path.

        Splits ``module.submodule.ClassName`` by the last dot, imports the
        module, retrieves the class, and instantiates it with **kwargs.
        """
        module_path, class_name = cls_path.rsplit(".", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)
        return cls(**kwargs)

    # ------------------------------------------------------------------
    # Sensor callbacks (fast, minimal work)
    # ------------------------------------------------------------------
    def _cb_rgb(self, msg):
        self.latest_rgb = msg

    def _cb_depth(self, msg):
        self.latest_depth = msg

    def _cb_cam_info(self, msg):
        self.latest_cam_info = msg

    # ------------------------------------------------------------------
    # Instruction callback — triggers detection in background thread
    # ------------------------------------------------------------------
    def _cb_instruction(self, msg):
        """Parse JSON instruction and spawn a detection thread."""
        try:
            data = json.loads(msg.data)
            target_text = data.get("target", {}).get("text", "")
        except (json.JSONDecodeError, KeyError) as e:
            rospy.logwarn("[detector] Malformed instruction: %s", msg.data[:100])
            return

        if not target_text:
            rospy.logwarn("[detector] Instruction has empty target text")
            return

        rospy.loginfo("[detector] Received instruction, target=%s", target_text)
        thread = threading.Thread(
            target=self._run_detection, args=(target_text,), daemon=True
        )
        thread.start()

    # ------------------------------------------------------------------
    # Detection pipeline (runs in background thread — never blocks ROS)
    # ------------------------------------------------------------------
    def _run_detection(self, target_text):
        """Full detection → TF → publish pipeline."""
        t_start = time.time()

        # Snapshot latest sensor data under lock
        with self.lock:
            rgb_msg = self.latest_rgb
            depth_msg = self.latest_depth
            cam_info = self.latest_cam_info

        if rgb_msg is None or depth_msg is None:
            rospy.logwarn(
                "[detector] Waiting for images (rgb=%s depth=%s)...",
                rgb_msg is not None,
                depth_msg is not None,
            )
            return

        try:
            # Convert ROS images to numpy
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

            # Convert BGR → RGB (Detector contract expects RGB)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            # Run detector
            detection = self.detector.detect(rgb, depth, target_text, cam_info)
            elapsed = time.time() - t_start

            if detection is not None:
                rospy.loginfo(
                    "[detector] Detected: bbox=%s center=%s depth=%.2f conf=%.2f",
                    detection.bbox,
                    detection.center,
                    detection.depth_m,
                    detection.confidence,
                )

                # TF transform camera → map
                point_map = self._transform_to_map(
                    detection.point_camera, rgb_msg.header.stamp
                )
                if point_map is not None:
                    ps = PointStamped(
                        header=Header(
                            stamp=rgb_msg.header.stamp,
                            frame_id=self.target_frame,
                        ),
                        point=Point(
                            x=point_map[0], y=point_map[1], z=point_map[2]
                        ),
                    )
                    self.pub_target.publish(ps)
                    rospy.loginfo(
                        "[detector] Published target: (%.2f, %.2f, %.2f) in %s",
                        point_map[0],
                        point_map[1],
                        point_map[2],
                        self.target_frame,
                    )

                debug_img = self._draw_debug(bgr, detection, elapsed)
            else:
                rospy.logwarn("[detector] Detection returned None")
                debug_img = self._draw_failure(bgr, "Detection returned None", elapsed)

            # Publish debug image
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

        except Exception as e:
            rospy.logerr("[detector] Detection error: %s", e)
            # Attempt to publish a failure debug image even on error
            try:
                with self.lock:
                    rgb_msg_err = self.latest_rgb
                if rgb_msg_err is not None:
                    bgr_err = self.bridge.imgmsg_to_cv2(rgb_msg_err, "bgr8")
                    t_elapsed = time.time() - t_start
                    fail_img = self._draw_failure(bgr_err, str(e), t_elapsed)
                    self.pub_debug.publish(
                        self.bridge.cv2_to_imgmsg(fail_img, "bgr8")
                    )
            except Exception:
                rospy.logerr("[detector] Could not publish failure debug image")

    # ------------------------------------------------------------------
    # TF: camera_optical → map
    # ------------------------------------------------------------------
    def _transform_to_map(self, point_camera, stamp):
        """Transform a 3D point from camera_optical frame to map frame.

        Args:
            point_camera: (x, y, z) tuple in camera_optical frame.
            stamp:        Timestamp for the TF lookup.

        Returns:
            (x, y, z) tuple in map frame, or None on TF failure.
        """
        ps_cam = PointStamped(
            header=Header(stamp=stamp, frame_id=self.camera_optical_frame),
            point=Point(x=point_camera[0], y=point_camera[1], z=point_camera[2]),
        )
        try:
            ps_map = self.tf_buffer.transform(
                ps_cam, self.target_frame, rospy.Duration(1.0)
            )
            return (ps_map.point.x, ps_map.point.y, ps_map.point.z)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("[detector] TF transform failed: %s", e)
            return None

    # ------------------------------------------------------------------
    # Debug image drawing
    # ------------------------------------------------------------------
    def _draw_debug(self, rgb, detection, elapsed):
        """Annotate image with detection bounding box, center cross, and info text.

        Args:
            rgb:       BGR image (numpy array, H×W×3).
            detection: Detection dataclass.
            elapsed:   Processing time in seconds.

        Returns:
            Annotated BGR image.
        """
        img = rgb.copy()
        x1, y1, x2, y2 = detection.bbox

        # Green rectangle around the detected region
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Red cross-hair at the bbox center
        cx, cy = detection.center
        cv2.line(img, (cx - 10, cy), (cx + 10, cy), (0, 0, 255), 2)
        cv2.line(img, (cx, cy - 10), (cx, cy + 10), (0, 0, 255), 2)

        # White text with black outline (double-draw for readability)
        text = (
            f"target | depth={detection.depth_m:.2f}m "
            f"| conf={detection.confidence:.2f} "
            f"| t={elapsed:.2f}s"
        )
        cv2.putText(
            img, text, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
        )
        cv2.putText(
            img, text, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
        )

        return img

    def _draw_failure(self, rgb, reason, elapsed):
        """Annotate image with red failure message at bottom-center.

        Args:
            rgb:     BGR image (numpy array, H×W×3).
            reason:  Error description string.
            elapsed: Processing time in seconds.

        Returns:
            Annotated BGR image.
        """
        img = rgb.copy()
        text = f"DETECTION FAILED: {reason} | t={elapsed:.2f}s"
        h, w = img.shape[:2]
        (tw, th), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
        )
        x = (w - tw) // 2
        cv2.putText(
            img, text, (x, h - 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
        )
        return img


# ==============================================================================
# Main entry point
# ==============================================================================
if __name__ == "__main__":
    node = DetectorNode()
    rospy.spin()
