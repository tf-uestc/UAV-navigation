#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_manager.py — High-level mission state machine for UAV continuous search.

Manages the search lifecycle: IDLE -> SEARCHING -> APPROACHING -> HOVER,
coordinating VLM detection triggers, yaw sweeps, search patterns, and
geofenced goal publication.

States:
  IDLE        — Waiting for instruction or state command
  SEARCHING   — Yaw sweeps + lawnmower search with periodic VLM triggers
  APPROACHING — Following planner trajectory with periodic VLM refinement
  HOVER       — Holding position at target

Topics:
  Subscribe:
    /uav/instruction                     (std_msgs/String)                — JSON target spec
    /uav/target_world                    (geometry_msgs/PointStamped)     — Detection result
    /uav/planner_status                  (std_msgs/String)                — Planner health
    /uav/state_cmd                       (std_msgs/String)                — State commands
    /planning/bspline                    (ego_planner/Bspline)            — Planning health
    /mavros/local_position/pose          (geometry_msgs/PoseStamped)      — Current pose

  Publish:
    /uav/instruction                     (std_msgs/String)                — VLM re-trigger
    /uav/goal_pose                       (geometry_msgs/PoseStamped)      — Search setpoints
    /uav/trajectory                      (nav_msgs/Path)                  — Clear on entry

Params:
  ~search_height          (float) — Search altitude (default: 2.0)
  ~search_center_x        (float) — Search centre X (default: 0.0)
  ~search_center_y        (float) — Search centre Y (default: 0.0)
  ~search_vlm_interval    (float) — Seconds between VLM triggers during SEARCHING (default: 5.0)
  ~approach_vlm_interval  (float) — Seconds between VLM triggers during APPROACHING (default: 5.0)
  ~max_miss               (int)   — Max consecutive missed detections (default: 5)
  ~approach_distance      (float) — Distance to consider arrived at target (default: 2.0)
  ~max_search_rounds      (int)   — Yaw sweep rounds before switching to lawnmower (default: 3)
  ~geofence/x_min/x_max/y_min/y_max/z_min/z_max (float) — Geofence bounds
"""
import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import math
import threading
import time
import uuid
from functools import wraps

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header, String
from tf.transformations import quaternion_from_euler

from ego_planner.msg import Bspline
from nav_msgs.msg import Odometry
from navigation_core.targets import observation_goal, ArrivalTracker, TargetEstimator
from navigation_core.contracts import DetectionResult, vector3
from navigation_core.yaw import quaternion_yaw
from navigation_core.lifecycle import RequestRegistry, WaitBudget, CallBudget

# ==============================================================================
# Constants
# ==============================================================================
STATE_IDLE = "IDLE"
STATE_SEARCHING = "SEARCHING"
STATE_APPROACHING = "APPROACHING"
STATE_HOVER = "HOVER"

APPROACH_FLYING = "FLYING"
APPROACH_WAIT_VLM = "WAIT_VLM"
APPROACH_WAIT_PLAN = "WAIT_PLAN"

YAW_ANGLES_DEG = [0, 45, -45, 90, -90, 135, -135, 180]
YAW_SETTLE_TIME = 1.5  # seconds allowed for the drone to physically rotate
SEARCH_OFFSET = 4.0    # metres — lateral offset from search centre for each look-angle
MISSION_TRIGGER_KEY = "_mission_mgr_trigger"


def _serialized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._state_lock:
            return method(self, *args, **kwargs)
    return call


class MissionManager:
    """Mission state machine managing the search-to-approach lifecycle.

    The search loop runs in a background daemon thread and is interruptible
    via the ``_search_active`` flag. APPROACHING VLM re-triggers and arrival
    checks happen on a 10 Hz wall-clock supervisor.
    """

    def __init__(self):
        rospy.init_node("mission_manager")
        self._state_lock = threading.RLock()
        self._clock = time.monotonic
        self._mission_id = None
        self._last_ros_time = rospy.Time.now().to_sec()
        self._shutdown = threading.Event()
        self._requests = RequestRegistry()
        self._plan_wait = WaitBudget(rospy.get_param("~plan_retries", 2))
        self._plan_timeout = rospy.get_param("~plan_timeout", 12.)
        self._search_timeout = rospy.get_param("~search_vlm_timeout", 20.)
        self._search_budget = CallBudget(rospy.get_param("~max_api_calls", 200),
                                        rospy.get_param("~max_search_seconds", 600.))
        self._last_approach_mono = self._clock()
        self._plan_key = 0
        self._plan_requires_hold = True
        self._planner_ready = True
        self._last_plan_message = None
        self._hover_goal = None
        self._navigation_goal = None
        self._pose_mono = self._velocity_mono = None
        self._velocity = None
        self._arrival = ArrivalTracker()
        self._target_estimator = TargetEstimator()

        # ==================================================================
        # Parameters
        # ==================================================================
        self._search_height = rospy.get_param("~search_height", 2.0)
        self._search_center_x = rospy.get_param("~search_center_x", 0.0)
        self._search_center_y = rospy.get_param("~search_center_y", 0.0)
        self._search_vlm_interval = rospy.get_param("~search_vlm_interval", 5.0)
        self._approach_vlm_interval = rospy.get_param("~approach_vlm_interval", 5.0)
        self._approach_vlm_timeout = rospy.get_param("~approach_vlm_timeout", 20.0)
        self._pause_for_vlm = rospy.get_param("~pause_for_vlm_in_approach", True)
        self._max_miss = rospy.get_param("~max_miss", 5)
        self._approach_distance = rospy.get_param("~approach_distance", 2.0)
        self._max_search_rounds = rospy.get_param("~max_search_rounds", 3)

        # Geofence bounds
        self._geofence = {
            "x_min": rospy.get_param("~geofence/x_min", -100.0),
            "x_max": rospy.get_param("~geofence/x_max", 100.0),
            "y_min": rospy.get_param("~geofence/y_min", -100.0),
            "y_max": rospy.get_param("~geofence/y_max", 100.0),
            "z_min": rospy.get_param("~geofence/z_min", 0.5),
            "z_max": rospy.get_param("~geofence/z_max", 50.0),
        }

        # ==================================================================
        # State
        # ==================================================================
        self._state = STATE_IDLE
        self._current_pose = PoseStamped()       # latest /mavros/local_position/pose
        self._last_target = None                  # latest /uav/target_world
        self._target_text = ""                    # current search target description
        self._miss_count = 0
        self._search_round = 0
        self._search_active = False
        self._search_thread = None
        self._detection_in_flight = False
        self._last_instruction_time = rospy.Time(0)
        self._last_search_goal = (0.0, 0.0, 2.0, 0.0)  # (x, y, z, yaw) for continuous publish
        self._approach_phase = APPROACH_FLYING
        self._pending_detection_request_id = None
        self._next_request_id = 1
        self._last_approach_result_time = rospy.Time(0)
        self._vlm_wait_start_time = rospy.Time(0)
        self._approach_hold_goal = None
        self._last_valid_target = None
        self._waiting_plan_since = rospy.Time(0)
        self._trajectory_after_target = False

        # ==================================================================
        # Publishers
        # ==================================================================
        self._instruction_pub = rospy.Publisher(
            "/uav/instruction", String, queue_size=1,
        )
        self._goal_pub = rospy.Publisher(
            "/uav/goal_pose", PoseStamped, queue_size=10,
        )
        self._traj_pub = rospy.Publisher(
            "/uav/trajectory", PathMsg, queue_size=1, latch=True,
        )
        self._approach_target_pub = rospy.Publisher(
            "/uav/approach_target", PointStamped, queue_size=1, latch=True,
        )
        self._status_pub = rospy.Publisher(
            "/uav/mission_status", String, queue_size=10,
        )

        self._cancel_pub = rospy.Publisher("/uav/cancel_planning", String, queue_size=10)
        # ==================================================================
        # Subscribers
        # ==================================================================
        rospy.Subscriber(
            "/uav/instruction", String, self._on_instruction, queue_size=1,
        )
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._on_velocity, queue_size=1)
        rospy.Subscriber("/uav/detection_result", String, self._on_detection_result, queue_size=10)
        rospy.Subscriber(
            "/uav/target_world", PointStamped, self._on_target_world, queue_size=1,
        )
        rospy.Subscriber(
            "/uav/planner_status", String, self._on_planner_status, queue_size=1,
        )
        rospy.Subscriber(
            "/uav/detector_status", String, self._on_detector_status, queue_size=10,
        )
        rospy.Subscriber(
            "/uav/state_cmd", String, self._on_state_cmd, queue_size=10,
        )
        rospy.Subscriber(
            "/planning/bspline", Bspline, self._on_bspline, queue_size=1,
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10,
        )
        rospy.Subscriber(
            "/uav/trajectory", PathMsg, self._on_trajectory, queue_size=1,
        )

        rospy.loginfo("[mission] MissionManager initialized (state=%s)", self._state)

    # ======================================================================
    # Utility: interruptible sleep
    # ======================================================================
    def _sleep(self, duration):
        """Sleep for *duration* seconds, interrupting early when
        ``_search_active`` is cleared or ROS shuts down.

        Checks the flags every 0.1 s so the background search thread
        exits promptly on state transitions.
        """
        step = 0.1
        elapsed = 0.0
        while elapsed < duration and self._search_active and not rospy.is_shutdown():
            rospy.sleep(step)
            elapsed += step

    # ======================================================================
    # Geofence clipping
    # ======================================================================
    @staticmethod
    def _clip_to_geofence(x, y, z, gf):
        """Return (x, y, z) clipped to the geofence dict *gf*.

        Args:
            x, y, z:  Coordinates to clip.
            gf:       Geofence dict with keys x_min, x_max, y_min, y_max,
                      z_min, z_max.

        Returns:
            (x, y, z) tuple clamped to the fence.
        """
        return (
            max(gf["x_min"], min(gf["x_max"], x)),
            max(gf["y_min"], min(gf["y_max"], y)),
            max(gf["z_min"], min(gf["z_max"], z)),
        )

    # ======================================================================
    # Search pattern generators (inline static methods)
    # ======================================================================
    @staticmethod
    def _gen_lawnmower_waypoints(center, width, height, spacing, altitude):
        """Generate a boustrophedon (lawnmower) search pattern.

        The pattern sweeps back-and-forth along the Y axis, alternating
        direction on each pass.

        Args:
            center:   (x, y) tuple for the centre of the search area.
            width:    Total width of the search area in X (m).
            height:   Total height of the search area in Y (m).
            spacing:  Spacing between adjacent passes (m).
            altitude: Z altitude for all waypoints.

        Returns:
            List of (x, y, z) tuples.
        """
        cx, cy = center
        half_w = width / 2.0
        half_h = height / 2.0
        y_start = cy - half_h
        y_end = cy + half_h

        num_passes = max(1, int(math.ceil(height / spacing)))
        step = height / num_passes if num_passes > 0 else 0.0

        waypoints = []
        for i in range(num_passes + 1):
            y = y_start + i * step
            y = min(y, y_end)  # clamp to boundary
            if i % 2 == 0:
                # Even pass: left -> right
                waypoints.append((cx - half_w, y, altitude))
                waypoints.append((cx + half_w, y, altitude))
            else:
                # Odd pass: right -> left
                waypoints.append((cx + half_w, y, altitude))
                waypoints.append((cx - half_w, y, altitude))

        return waypoints

    @staticmethod
    def _gen_spiral_waypoints(center, max_radius, spacing, altitude):
        """Generate an outward spiral search pattern.

        Starts at the centre and spirals outward. The number of points per
        ring scales with the ring radius so the angular spacing stays
        approximately constant.

        Args:
            center:    (x, y) tuple for the centre of the spiral.
            max_radius: Maximum radius of the spiral (m).
            spacing:   Radial spacing between spiral rings (m).
            altitude:  Z altitude for all waypoints.

        Returns:
            List of (x, y, z) tuples.
        """
        cx, cy = center
        waypoints = [(cx, cy, altitude)]  # start at centre
        num_rings = max(1, int(math.ceil(max_radius / spacing)))

        for ring in range(1, num_rings + 1):
            r = ring * spacing
            # At least 8 points per ring, scale with circumference
            n_pts = max(8, int(2.0 * math.pi * r / spacing))
            for j in range(n_pts):
                theta = 2.0 * math.pi * j / n_pts
                x = cx + r * math.cos(theta)
                y = cy + r * math.sin(theta)
                waypoints.append((x, y, altitude))

        return waypoints

    # ======================================================================
    # ROS callbacks
    # ======================================================================
    def _on_pose(self, msg):
        try:
            vector3((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
            q = msg.pose.orientation
            quaternion_yaw((q.x, q.y, q.z, q.w))
            if msg.header.frame_id != "map":
                raise ValueError("pose frame must be map")
        except ValueError:
            with self._state_lock:
                self._pose_mono = None
            return
        with self._state_lock:
            self._current_pose = msg
            self._pose_mono = self._clock()

    def _on_velocity(self, msg):
        v = msg.twist.twist.linear
        try:
            velocity = vector3((v.x, v.y, v.z))
        except ValueError:
            return
        with self._state_lock:
            # Arrival uses the rotation-invariant speed norm, not body velocity direction.
            self._velocity = velocity
            self._velocity_mono = self._clock()

    def _on_bspline(self, msg):
        """Monitor EGO-Planner health (minimal — subscription keeps the
        connection alive for status awareness)."""
        pass

    @_serialized
    def _on_instruction(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("object required")
            if data.get(MISSION_TRIGGER_KEY, False):
                return
            target = data.get("target", {}).get("text", "")
            if not isinstance(target, str) or not target.strip():
                raise ValueError("target required")
        except (ValueError, TypeError, AttributeError):
            rospy.logwarn("[mission] Invalid instruction")
            return
        self._last_ros_time = rospy.Time.now().to_sec()
        self._mission_id = uuid.uuid4().hex
        self._requests.begin(self._mission_id)
        self._search_budget.begin(self._clock())
        self._plan_wait.cancel()
        self._cancel_pub.publish(String(data=self._mission_id))
        self._target_text = target
        self._last_target = self._last_valid_target = None
        self._navigation_goal = None
        self._target_estimator.reset()
        self._miss_count = self._search_round = 0
        self._detection_in_flight = False
        self._pending_detection_request_id = None
        if self._state == STATE_SEARCHING:
            self._clear_trajectory()
        self._set_state(STATE_SEARCHING)

    def _on_target_world(self, msg):
        # Bare PointStamped carries no reliable task/request identity.
        # It remains a visualization/direct-mode topic, never a managed result.
        rospy.loginfo_throttle(5., "[mission] Ignoring legacy target_world; awaiting identified result")

    @_serialized
    def _on_detection_result(self, msg):
        try:
            result = DetectionResult.from_json(msg.data)
        except (ValueError, TypeError, KeyError):
            rospy.logwarn("[mission] Invalid detection result")
            return
        if not self._check_clock_epoch():
            return
        if self._state not in (STATE_SEARCHING, STATE_APPROACHING):
            return
        if not self._requests.resolve(result, self._clock(),
                                      rospy.Time.now().to_sec(), self._approach_vlm_timeout + 5.):
            rospy.loginfo("[mission] Rejected result: %s", self._requests.reason)
            if self._requests.reason == "timeout":
                self._detection_in_flight = False
                self._pending_detection_request_id = None
                if self._state == STATE_APPROACHING:
                    self._handle_approach_vlm_timeout()
                else:
                    self._miss_count += 1
            return
        self._detection_in_flight = False
        self._pending_detection_request_id = None
        if result.status != "SUCCEEDED":
            if self._state == STATE_APPROACHING:
                self._handle_approach_vlm_timeout()
            else:
                self._miss_count += 1
            return
        if not self._target_estimator.observe(result.point, result.capture_stamp, rospy.Time.now().to_sec()):
            if self._state == STATE_APPROACHING:
                self._handle_approach_vlm_timeout()
            else:
                self._miss_count += 1
            return
        self._miss_count = 0
        target = PointStamped(header=Header(stamp=rospy.Time.from_sec(result.capture_stamp), frame_id="map"),
                              point=Point(*result.point))
        self._last_target = self._last_valid_target = target
        self._last_approach_mono = self._clock()
        if self._state == STATE_SEARCHING:
            self._set_state(STATE_APPROACHING)
        elif (not self._pause_for_vlm and self._approach_phase == APPROACH_WAIT_VLM
              and not self._target_estimator.needs_replan()):
            # Keep the active path for small refinements. A paused vehicle still
            # needs a newly planned path from its measured hold position.
            self._approach_phase = APPROACH_FLYING
        else:
            self._publish_validated_target(target, hold=self._pause_for_vlm)

    def _check_clock_epoch(self):
        now = rospy.Time.now().to_sec()
        if now < self._last_ros_time:
            self._last_ros_time = now
            self._set_state(STATE_HOVER)
            return False
        self._last_ros_time = now
        return True

    def _begin_detection(self, approach):
        now = self._clock()
        expired = self._requests.expire(now)
        if expired:
            self._detection_in_flight = False
            self._pending_detection_request_id = None
            self._miss_count += 1
        if self._requests.pending is not None:
            return False
        if not self._mission_id or not self._target_text:
            return False
        if not self._search_budget.reserve(now):
            self._set_state(STATE_HOVER)
            return False
        timeout = self._approach_vlm_timeout if approach else self._search_timeout
        key = self._requests.issue(self._target_text, now, timeout)
        self._detection_in_flight = True
        self._pending_detection_request_id = key.request_id
        self._last_instruction_time = rospy.Time.now()
        if approach:
            self._approach_phase = APPROACH_WAIT_VLM
            self._vlm_wait_start_time = rospy.Time.now()
        data = {"target": {"text": self._target_text}, MISSION_TRIGGER_KEY: True,
                "mission_id": key.mission_id, "request_id": key.request_id}
        # Register every state/ID before publishing; an immediate BUSY is valid.
        self._instruction_pub.publish(String(data=json.dumps(data)))
        return True

    def _on_detector_status(self, msg):
        # Diagnostics only. Terminal business results have one authoritative topic.
        return

    @_serialized
    def _on_trajectory(self, msg):
        if self._state != STATE_APPROACHING or self._approach_phase != APPROACH_WAIT_PLAN or not msg.poses:
            return
        # Compatibility Path has no goal ID. U05 must add actual EGO acknowledgement.
        if msg.header.stamp < self._waiting_plan_since:
            return
        self._plan_wait.complete(self._plan_key)
        self._trajectory_after_target = True
        self._approach_phase = APPROACH_FLYING
        self._last_approach_mono = self._clock()
        self._approach_hold_goal = None

    @_serialized
    def _on_planner_status(self, msg):
        status = msg.data.strip()
        if status == "MAP_NOT_READY":
            self._planner_ready = False
        elif status == "OK":
            self._planner_ready = True
        elif status == "FAIL" and self._state == STATE_APPROACHING:
            self._set_state(STATE_HOVER)

    @_serialized
    def _on_state_cmd(self, msg):
        """Handle external state commands.

        Accepted commands:
          SEARCH / START  — Begin searching (from IDLE).
          HOVER            — Hold current position.
          LAND / RTL       — Return to IDLE (bridge handles landing).
        """
        cmd = msg.data.strip().upper()
        rospy.loginfo("[mission] State command: %s", cmd)

        if cmd in ("SEARCH", "START"):
            if self._state == STATE_IDLE:
                if not self._target_text:
                    rospy.logwarn("[mission] START requires a target instruction")
                    return
                self._on_instruction(String(data=json.dumps({"target": {"text": self._target_text}})))
            else:
                rospy.loginfo("[mission] Already active (%s) — ignoring %s", self._state, cmd)
        elif cmd == "HOVER":
            self._set_state(STATE_HOVER)
        elif cmd in ("LAND", "RTL", "EMERGENCY"):
            rospy.loginfo("[mission] %s — returning to IDLE", cmd)
            self._set_state(STATE_IDLE)

    # ======================================================================
    # State transitions
    # ======================================================================
    @_serialized
    def _set_state(self, new_state):
        """Transition to *new_state*, executing exit and entry actions."""
        if new_state == self._state:
            return
        old_state = self._state
        rospy.loginfo("[mission] State: %s -> %s", old_state, new_state)

        # --- Exit current state ---
        if old_state == STATE_SEARCHING:
            self._exit_searching()
        elif old_state == STATE_APPROACHING:
            self._exit_approaching()

        # --- Enter new state ---
        self._state = new_state
        self._status_pub.publish(String(data=self._state))
        if new_state == STATE_SEARCHING:
            self._enter_searching()
        elif new_state == STATE_APPROACHING:
            self._enter_approaching()
        elif new_state == STATE_HOVER:
            self._enter_hover()
        elif new_state == STATE_IDLE:
            self._enter_idle()

    def _enter_idle(self):
        self._search_active = False
        self._requests.cancel()
        self._plan_wait.cancel()
        self._detection_in_flight = False
        self._pending_detection_request_id = None
        self._clear_trajectory()
        self._cancel_pub.publish(String(data=self._mission_id or "cancel"))

    def _enter_searching(self):
        """Start the background search thread and clear any active trajectory.

        The empty trajectory causes ``trajectory_follower`` to stop
        publishing to ``/uav/goal_pose``, so the mission manager's
        search setpoints reach the setpoint bridge uncontested.
        """
        self._clear_trajectory()
        self._miss_count = 0
        self._search_round = 0
        self._search_active = True
        self._pending_detection_request_id = None
        self._approach_hold_goal = None

        if self._search_thread is None or not self._search_thread.is_alive():
            self._search_thread = threading.Thread(
                target=self._search_loop, daemon=True,
                name="mission_search",
            )
            self._search_thread.start()
            rospy.loginfo("[mission] Started search thread")

        rospy.loginfo("[mission] Entered SEARCHING (target=%s)", self._target_text)

    def _exit_searching(self):
        """Stop the search thread and reset search state."""
        self._search_active = False
        rospy.loginfo("[mission] Exited SEARCHING")

    def _enter_approaching(self):
        """Set up for the approach phase.

        The planner generates a trajectory and ``trajectory_follower``
        handles the low-level setpoints. The mission manager pauses at
        fixed intervals to trigger VLM refinement, then waits for a new
        trajectory before flying again.
        """
        self._approach_phase = APPROACH_WAIT_PLAN
        self._last_approach_result_time = rospy.Time.now()
        self._detection_in_flight = False
        self._pending_detection_request_id = None
        self._trajectory_after_target = False
        self._waiting_plan_since = rospy.Time.now()
        self._capture_current_hold_goal()
        if self._last_target is not None:
            self._publish_validated_target(self._last_target)
        rospy.loginfo("[mission] Entered APPROACHING (phase=%s)", self._approach_phase)

    def _exit_approaching(self):
        """Clean up approach state."""
        self._detection_in_flight = False
        self._pending_detection_request_id = None
        self._approach_hold_goal = None
        rospy.loginfo("[mission] Exited APPROACHING")

    def _enter_hover(self):
        self._requests.cancel()
        self._plan_wait.cancel()
        self._detection_in_flight = False
        self._pending_detection_request_id = None
        self._clear_trajectory()
        self._cancel_pub.publish(String(data=self._mission_id or "cancel"))
        self._capture_current_hold_goal()
        self._hover_goal = self._approach_hold_goal
        self._publish_hold_goal()

    # ======================================================================
    # Core operations
    # ======================================================================
    def _clear_trajectory(self):
        """Publish an empty Path to clear the latched /uav/trajectory.

        When ``trajectory_follower`` receives an empty path its
        output stops, freeing ``/uav/goal_pose`` for the mission
        manager's search setpoints.
        """
        empty_path = PathMsg()
        empty_path.header = Header(
            stamp=rospy.Time.now(), frame_id="map",
        )
        self._traj_pub.publish(empty_path)
        rospy.loginfo("[mission] Cleared trajectory (empty Path published)")

    def _publish_goal(self, x, y, z, yaw=0.0, cache_search=True):
        """Publish a geofence-clipped position setpoint to /uav/goal_pose.

        Args:
            x, y, z: Target position (clipped to geofence before publishing).
            yaw: Target yaw in radians (default 0.0).
        """
        cx, cy, cz = self._clip_to_geofence(x, y, z, self._geofence)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose = PoseStamped()
        pose.header = Header(stamp=rospy.Time.now(), frame_id="map")
        pose.pose.position = Point(x=cx, y=cy, z=cz)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self._goal_pub.publish(pose)
        if cache_search:
            # Cache for continuous re-publish during SEARCHING (defeats goal_timeout)
            self._last_search_goal = (cx, cy, cz, yaw)

    def _capture_current_hold_goal(self):
        """Cache the current pose as a hold setpoint for approach pauses."""
        pose = self._current_pose.pose
        q = pose.orientation
        sy = 2.0 * (q.w * q.z + q.x * q.y)
        cy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sy, cy)
        p = pose.position
        self._approach_hold_goal = (p.x, p.y, p.z, yaw)

    def _publish_hold_goal(self):
        """Continuously publish the cached hold goal during WAIT phases."""
        if self._approach_hold_goal is None:
            self._capture_current_hold_goal()
        x, y, z, yaw = self._approach_hold_goal
        self._publish_goal(x, y, z, yaw=yaw, cache_search=False)

    def _pause_approach_for_vlm(self):
        """Stop trajectory playback and hold current pose before VLM."""
        self._clear_trajectory()
        self._capture_current_hold_goal()

    def _publish_validated_target(self, msg, hold=True):
        actual = self._current_pose.pose.position
        try:
            if self._pose_mono is None or self._clock()-self._pose_mono > 1.:
                raise ValueError("current pose unavailable")
            goal = observation_goal((actual.x, actual.y, actual.z),
                                    (msg.point.x, msg.point.y, msg.point.z),
                                    self._search_height, self._approach_distance, self._geofence)
        except ValueError as exc:
            rospy.logwarn("[mission] Invalid navigation goal: %s", exc)
            self._set_state(STATE_HOVER)
            return
        self._plan_key += 1
        self._approach_phase = APPROACH_WAIT_PLAN
        self._plan_requires_hold = hold
        self._trajectory_after_target = False
        self._waiting_plan_since = rospy.Time.now()
        self._plan_wait.enter(self._plan_key, self._clock(), self._plan_timeout)
        target = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="map"),
                              point=Point(*goal.position))
        self._navigation_goal = goal.position
        self._target_estimator.mark_sent()
        self._last_plan_message = target
        self._approach_target_pub.publish(target)

    def _is_expected_detection(self, msg):
        return False  # Bare PointStamped cannot prove business identity.

    @_serialized
    def _trigger_vlm(self):
        return self._begin_detection(False)

    @_serialized
    def _trigger_approach_vlm(self):
        return self._begin_detection(True)

    def _handle_approach_vlm_timeout(self):
        # Expiry/terminal handlers consume the registry entry before arriving here.
        self._detection_in_flight = False
        self._pending_detection_request_id = None
        self._miss_count += 1
        self._last_approach_mono = self._clock()
        if self._miss_count >= self._max_miss:
            self._set_state(STATE_HOVER)
        elif self._last_valid_target is not None:
            self._publish_validated_target(self._last_valid_target, hold=self._pause_for_vlm)
        else:
            self._set_state(STATE_HOVER)

    def _check_approach_arrival(self):
        now = self._clock()
        if (self._navigation_goal is None or self._velocity is None or
                self._pose_mono is None or self._velocity_mono is None or
                now-self._pose_mono > 1. or now-self._velocity_mono > 1.):
            return False
        p = self._current_pose.pose.position
        return self._arrival.update((p.x, p.y, p.z), self._velocity, self._navigation_goal, now)

    # ======================================================================
    # Search loop (runs in a background daemon thread — NEVER blocks ROS)
    # ======================================================================
    def _search_loop(self):
        """Background daemon thread running the search mission.

        **Phase 1 — Yaw sweeps**: up to ``max_search_rounds`` full rotations
        at the search centre (``search_center_x/y``, ``search_height``),
        triggering VLM at each of the eight cardinal/intercardinal headings.

        **Phase 2 — Lawnmower**: if the yaw sweeps exhaust without finding
        a target the drone switches to a boustrophedon pattern covering a
        30 x 30 m area centred on the search centre.

        The loop checks ``_search_active`` and ``_state`` at every
        opportunity so it exits promptly when a target is found or the
        mission is cancelled.
        """
        rospy.loginfo("[mission] Search thread started")

        sx = self._search_center_x
        sy = self._search_center_y
        sz = self._search_height

        # Fly to the search centre first
        self._publish_goal(sx, sy, sz, yaw=0.0)
        self._sleep(2.0)  # allow time to reach altitude

        while self._search_active and not rospy.is_shutdown():
            if self._state != STATE_SEARCHING:
                self._sleep(0.1)
                continue

            # --- Phase 1: Yaw sweeps at the search centre ---
            if self._search_round < self._max_search_rounds:
                rospy.loginfo(
                    "[mission] Yaw sweep round %d/%d",
                    self._search_round + 1, self._max_search_rounds,
                )
                for yaw_deg in YAW_ANGLES_DEG:
                    if not self._search_active or self._state != STATE_SEARCHING:
                        rospy.loginfo("[mission] Search interrupted during yaw sweep")
                        return
                    yaw_rad = math.radians(yaw_deg)
                    # In-place yaw rotation at the search centre
                    self._publish_goal(sx, sy, sz, yaw=yaw_rad)
                    rospy.loginfo("[mission] Yaw sweep %3d deg", yaw_deg)

                    # Let the drone physically rotate
                    self._sleep(YAW_SETTLE_TIME)

                    # Trigger VLM detection
                    self._trigger_vlm()

                    # Wait for the detection to complete (vlm_interval)
                    self._sleep(self._search_vlm_interval)

                    # If a target was found during this interval the state
                    # transitioned to APPROACHING — exit immediately
                    if self._state != STATE_SEARCHING:
                        return

                self._search_round += 1

            else:
                # --- Phase 2: Lawnmower search ---
                rospy.loginfo(
                    "[mission] Yaw sweeps exhausted — switching to lawnmower pattern",
                )
                self._run_lawnmower_search((sx, sy))

                # After one full lawnmower pass reset the round counter so
                # the loop performs another round of yaw sweeps at the new
                # location (or back at the centre) before lawnmowering again
                self._search_round = 0

        rospy.loginfo("[mission] Search thread exiting")

    def _run_lawnmower_search(self, centre):
        """Execute a single pass of the lawnmower pattern.

        At each waypoint the drone pauses briefly and triggers VLM. If
        the target is found during a pause the state machine transitions
        to APPROACHING and this method returns early.

        Args:
            centre: (x, y) tuple for the centre of the search area.
        """
        waypoints = self._gen_lawnmower_waypoints(
            centre, width=30.0, height=30.0, spacing=5.0,
            altitude=self._search_height,
        )
        rospy.loginfo(
            "[mission] Lawnmower search: %d waypoints", len(waypoints),
        )

        for i, (wx, wy, wz) in enumerate(waypoints):
            if not self._search_active or self._state != STATE_SEARCHING:
                return

            rospy.loginfo(
                "[mission] Lawnmower waypoint %d/%d: (%.1f, %.1f, %.1f)",
                i + 1, len(waypoints), wx, wy, wz,
            )

            # Fly to the waypoint
            self._publish_goal(wx, wy, wz, yaw=0.0)
            self._sleep(3.0)  # allow time to reach the waypoint

            # Trigger VLM detection at this location
            self._trigger_vlm()
            self._sleep(self._search_vlm_interval)

            if self._state != STATE_SEARCHING:
                return

    # ======================================================================
    # Housekeeping timer (ROS spinner thread — 10 Hz)
    # ======================================================================
    @_serialized
    def _on_housekeeping_tick(self, event):
        if not self._check_clock_epoch():
            return
        now = self._clock()
        if self._state == STATE_SEARCHING:
            # Search movement still uses the legacy path until U08.
            if self._requests.expire(now):
                self._miss_count += 1
                self._detection_in_flight = False
                self._pending_detection_request_id = None
            if self._search_budget.started is not None and now-self._search_budget.started >= self._search_budget.max_seconds:
                self._set_state(STATE_HOVER)
                return
            self._publish_goal(*self._last_search_goal[:3], yaw=self._last_search_goal[3])
            return
        if self._state == STATE_HOVER:
            if self._hover_goal is not None:
                self._approach_hold_goal = self._hover_goal
                self._publish_hold_goal()
            return
        if self._state != STATE_APPROACHING:
            return
        if self._approach_phase == APPROACH_FLYING:
            if now-self._last_approach_mono >= self._approach_vlm_interval:
                if self._pause_for_vlm:
                    self._pause_approach_for_vlm()
                self._trigger_approach_vlm()
                if (self._state == STATE_APPROACHING and
                        self._approach_phase in (APPROACH_WAIT_VLM, APPROACH_WAIT_PLAN) and self._pause_for_vlm):
                    self._publish_hold_goal()
        elif self._approach_phase == APPROACH_WAIT_VLM:
            if self._requests.expire(now):
                self._handle_approach_vlm_timeout()
                return
            if self._pause_for_vlm:
                self._publish_hold_goal()
        elif self._approach_phase == APPROACH_WAIT_PLAN:
            action = self._plan_wait.tick(now, self._planner_ready)
            if action == "FAILED":
                self._set_state(STATE_HOVER)
                return
            if action == "RETRY" and self._last_plan_message is not None:
                self._approach_target_pub.publish(self._last_plan_message)
            if self._approach_phase == APPROACH_WAIT_PLAN and self._plan_requires_hold:
                self._publish_hold_goal()
        if self._check_approach_arrival():
            self._set_state(STATE_HOVER)
        # U08 validates these speed/dwell decisions against physical SITL data.

    # ======================================================================
    # Run
    # ======================================================================
    def run(self):
        """Wall-clock request supervision must keep running while /clock pauses."""
        def supervise():
            while not rospy.is_shutdown() and not self._shutdown.is_set():
                self._on_housekeeping_tick(None)
                self._shutdown.wait(.1)
        rospy.on_shutdown(self._shutdown.set)
        threading.Thread(target=supervise, daemon=True, name="mission_housekeeping").start()
        rospy.spin()


# ==============================================================================
if __name__ == "__main__":
    node = MissionManager()
    node.run()
