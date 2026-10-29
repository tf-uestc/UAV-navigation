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
import tf2_geometry_msgs  # Required for PointStamped transform support
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String

# Ensure scripts/ directory is on PYTHONPATH so detector modules are importable
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from base_detector import Detection, Detector
from navigation_core.contracts import DetectionResult, RequestKey, vector3
from navigation_core.perception import normalize_depth, validate_snapshot, validate_intrinsics


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
        self._detecting = False
        self._active_request_id = None
        self._active_mission_id = None
        self._detect_lock = threading.Lock()

        self._managed_only = rospy.get_param('~managed_requests_only', False)
        self._max_skew = rospy.get_param('~max_image_skew', .05)
        self._max_image_age = rospy.get_param('~max_image_age', 1.0)

        # --- API key resolution ---
        vlm_api_key = rospy.get_param("~vlm_api_key", "")
        if not vlm_api_key:
            vlm_api_key = os.environ.get("VLM_API_KEY", "")

        # --- Load detector via importlib ---
        cls_path = rospy.get_param("~detector_class", "qwen_vl_grounding.QwenVLGroundingDetector")
        detector_args = rospy.get_param("~detector_args", {})
        if vlm_api_key and "api_key" not in detector_args:
            detector_args["api_key"] = vlm_api_key
        vlm_model = rospy.get_param("~vlm_model", "")
        if vlm_model and "model" not in detector_args:
            detector_args["model"] = vlm_model
        vlm_timeout = rospy.get_param("~vlm_timeout", None)
        if vlm_timeout is not None and "timeout" not in detector_args:
            detector_args["timeout"] = vlm_timeout
        self.detector = self._load_detector(cls_path, detector_args)
        rospy.loginfo("[detector] Loaded detector: %s", cls_path)

        # --- TF ---
        # Buffer 120s to retain TF history for VLM API delays (up to ~90s)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(120.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.camera_optical_frame = rospy.get_param(
            "~camera_optical_frame", "camera_link_optical"
        )

        # --- Reserved future feature ---
        trigger_on_start = rospy.get_param("~trigger_on_start", False)
        if trigger_on_start:
            rospy.loginfo("[detector] trigger_on_start=true (future feature)")

        # --- Publishers ---
        self.pub_target = rospy.Publisher(
            "/uav/target_world", PointStamped, queue_size=1, latch=False
        )
        self.pub_debug = rospy.Publisher("/uav/target_debug", Image, queue_size=1, latch=True)
        self.pub_status = rospy.Publisher(
            "/uav/detector_status", String, queue_size=10
        )

        self.pub_result = rospy.Publisher("/uav/detection_result", String, queue_size=10)

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
        with self.lock:
            self.latest_rgb = msg

    def _cb_depth(self, msg):
        with self.lock:
            self.latest_depth = msg

    def _cb_cam_info(self, msg):
        with self.lock:
            self.latest_cam_info = msg

    def _cb_instruction(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("instruction must be an object")
            target_text = data.get("target", {}).get("text", "")
            if not isinstance(target_text, str) or not target_text.strip():
                raise ValueError("target text required")
            if self._managed_only and not data.get("_mission_mgr_trigger", False):
                return
            mission_id = data.get("mission_id", "")
            request_id = data.get("request_id", 0)
            if mission_id:
                RequestKey(mission_id, request_id)
            elif self._managed_only:
                raise ValueError("managed request requires business IDs")
        except (ValueError, TypeError, AttributeError, KeyError):
            rospy.logwarn("[detector] Invalid instruction")
            return
        with self._detect_lock:
            busy = self._detecting
            if not busy:
                self._detecting = True
                self._active_request_id, self._active_mission_id = request_id, mission_id
        if busy:
            self._publish_terminal(mission_id, request_id, target_text, "BUSY", 0., None, "detector_busy")
            return
        self._publish_status(request_id, "STARTED", mission_id)
        threading.Thread(target=self._run_detection,
                         args=(target_text, request_id, mission_id), daemon=True).start()

    def _run_detection(self, target_text, request_id=0, mission_id=""):
        started = time.monotonic()
        capture = 0.
        point = None
        status, reason = "FAILED", "no_observation"
        bgr = None
        try:
            with self.lock:
                rgb_msg, depth_msg, info = self.latest_rgb, self.latest_depth, self.latest_cam_info
            if rgb_msg is None or depth_msg is None or info is None:
                raise ValueError("sensor_data_unavailable")
            capture = rgb_msg.header.stamp.to_sec()
            if capture <= 0:
                raise ValueError("camera_clock_not_initialized")
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = normalize_depth(self.bridge.imgmsg_to_cv2(depth_msg, "passthrough"), depth_msg.encoding)
            validate_snapshot(capture, depth_msg.header.stamp.to_sec(), rospy.Time.now().to_sec(),
                              bgr.shape, depth.shape, rgb_msg.header.frame_id, depth_msg.header.frame_id,
                              self._max_skew, self._max_image_age)
            validate_intrinsics(info.K)
            if info.width != depth.shape[1] or info.height != depth.shape[0]:
                raise ValueError("camera_info_size_mismatch")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            detection = self.detector.detect(rgb, depth, target_text, info)
            if detection is None:
                reason = "no_valid_detection"
            else:
                point = self._transform_to_map(detection.point_camera, rgb_msg.header.stamp)
                if point is None:
                    reason = "historical_tf_unavailable"
                else:
                    point = vector3(tuple(float(v) for v in point))
                    status, reason = "SUCCEEDED", ""
                    legacy = PointStamped(header=Header(stamp=rgb_msg.header.stamp, frame_id="map"),
                                          point=Point(*point))
                    self.pub_target.publish(legacy)  # Visualization/direct legacy mode only.
                try:
                    self.pub_debug.publish(self.bridge.cv2_to_imgmsg(
                        self._draw_debug(bgr, detection, time.monotonic()-started), "bgr8"))
                except Exception:
                    rospy.logwarn("[detector] Debug image unavailable")
        except Exception as exc:
            status, point = "FAILED", None
            reason = str(exc)
            rospy.logwarn("[detector] Observation rejected: %s", reason)
        finally:
            try:
                self._publish_terminal(mission_id, request_id, target_text, status, capture, point, reason)
            finally:
                with self._detect_lock:
                    if (self._active_mission_id, self._active_request_id) == (mission_id, request_id):
                        self._detecting = False
                        self._active_request_id = self._active_mission_id = None

    def _publish_terminal(self, mission_id, request_id, target_text, status, capture, point, reason):
        completed = rospy.Time.now().to_sec()
        if completed < capture:
            status, capture, point, reason = "FAILED", 0., None, "clock_reset"
        if mission_id:
            result = DetectionResult(RequestKey(mission_id, request_id), target_text, status,
                                     capture, completed, point, "map", reason)
            self.pub_result.publish(String(data=result.to_json()))
        self._publish_status(request_id, status, mission_id)

    def _publish_status(self, request_id, status, mission_id=""):
        self.pub_status.publish(String(data=json.dumps(
            {"mission_id": mission_id, "request_id": request_id, "status": status})))

    def _transform_to_map(self, point_camera, stamp):
        if stamp.to_sec() <= 0:
            return None
        point = PointStamped(header=Header(stamp=stamp, frame_id=self.camera_optical_frame),
                             point=Point(*point_camera))
        try:
            transformed = self.tf_buffer.transform(point, self.target_frame, rospy.Duration(2.))
            return (transformed.point.x, transformed.point.y, transformed.point.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # An old image must never be transformed with the current camera pose.
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
