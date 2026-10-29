"""planning/ego_adapter.py — EgoPlannerAdapter wrapping EGO-Planner.

Publishes goals to /move_base_simple/goal, subscribes to EGO's
/planning/bspline output, and converts it to the Trajectory format
used by planner_node.
"""

import time as _time
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from navigation_core.bspline import BSpline
from navigation_core.yaw import YawPolicy, quaternion_yaw
from std_msgs.msg import Header

# EGO custom message — may need ego_planner workspace sourced
try:
    from ego_planner.msg import Bspline
    HAS_BSPLINE_MSG = True
except ImportError:
    HAS_BSPLINE_MSG = False
    Bspline = None  # type: ignore

from planning.base import Planner, Trajectory, TrajectoryPoint


# ---------------------------------------------------------------------------
# EgoPlannerAdapter
# ---------------------------------------------------------------------------
class EgoPlannerAdapter(Planner):
    """Bridge between the ROS Planner ABC and EGO-Planner.

    EGO-Planner is a C++ node, not a library.  This adapter communicates
    with it via ROS topics:

        input  →  /move_base_simple/goal  (PoseStamped)
        output ←  /planning/bspline        (ego_planner/Bspline)

    On plan(), the adapter publishes the goal and blocks up to
    ``plan_timeout`` seconds for EGO to produce a B-spline trajectory.
    """

    def __init__(self, plan_timeout: float = 10.0) -> None:
        self._plan_timeout = plan_timeout
        self._last_traj: Optional[Trajectory] = None
        self._odom_received = False
        self._map_ready = False
        self._current_yaw = None
        self._goal_position = None

        # --- Subscribe to EGO output ---
        if HAS_BSPLINE_MSG:
            self._bspline_sub = rospy.Subscriber(
                "/planning/bspline", Bspline,
                self._on_bspline, queue_size=1,
            )
        else:
            rospy.logwarn(
                "[ego_adapter] ego_planner/Bspline msg not available — "
                "make sure ego_planner workspace is sourced"
            )

        # --- Publish goals to EGO ---
        self._goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1,
        )

        # --- Listen for odometry to know EGO pipeline is alive ---
        rospy.Subscriber(
            "/mavros/local_position/odom",
            Odometry,
            self._on_odom,
            queue_size=1,
        )

        rospy.loginfo("[ego_adapter] EgoPlannerAdapter initialized")

    # ------------------------------------------------------------------
    # Planner ABC
    # ------------------------------------------------------------------
    def is_ready(self) -> bool:
        return self._odom_received

    def plan(self, start: Point, goal: Point) -> Optional[Trajectory]:
        """Send goal to EGO and block until a B-spline arrives."""
        self._last_traj = None
        self._make_goal(goal)
        deadline = _time.monotonic() + self._plan_timeout

        while not rospy.is_shutdown() and self._last_traj is None:
            if _time.monotonic() > deadline:
                rospy.logerr("[ego_adapter] Plan timed out after %.1f s",
                             self._plan_timeout)
                return None
            _time.sleep(0.05)

        return self._last_traj

    def shutdown(self) -> None:
        if HAS_BSPLINE_MSG and self._bspline_sub is not None:
            self._bspline_sub.unregister()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _on_odom(self, msg) -> None:
        q = msg.pose.pose.orientation
        try:
            self._current_yaw = quaternion_yaw((q.x, q.y, q.z, q.w))
            self._odom_received = True
        except ValueError:
            self._odom_received = False

    def _on_bspline(self, msg) -> None:
        if self._current_yaw is None:
            return
        try:
            self._last_traj = self._bspline_to_trajectory(
                msg, self._current_yaw, self._goal_position)
        except (ValueError, TypeError) as exc:
            rospy.logerr("[ego_adapter] Invalid B-spline: %s", exc)

    def _make_goal(self, goal: Point) -> None:
        self._goal_position = (goal.x, goal.y, goal.z)
        msg = PoseStamped()
        msg.header = Header(frame_id="map", stamp=rospy.Time.now())
        msg.pose.position = goal
        msg.pose.orientation = Quaternion(w=1.0)
        self._goal_pub.publish(msg)
        rospy.loginfo("[ego_adapter] Sent goal: (%.2f, %.2f, %.2f)",
                      goal.x, goal.y, goal.z)

    # ------------------------------------------------------------------
    # Bspline → Trajectory conversion
    # ------------------------------------------------------------------
    @staticmethod
    def _bspline_to_trajectory(msg, initial_yaw=0.0, goal_position=None) -> Trajectory:
        """Evaluate the actual spline and retain its original ROS start time.

        Goal/session acknowledgement and continuous forwarding require U05;
        this conversion alone does not establish closed-loop obstacle avoidance.
        """
        spline = BSpline([(p.x, p.y, p.z) for p in msg.pos_pts], msg.knots, msg.order)
        policy = YawPolicy(initial_yaw)
        points = []
        previous_t = 0.0
        for t, position, velocity, _acceleration in spline.samples():
            if goal_position is not None:
                yaw = policy.face(position, goal_position, t - previous_t)
            else:
                import math
                moving = math.hypot(velocity[0], velocity[1]) > 1e-6
                desired = math.atan2(velocity[1], velocity[0]) if moving else None
                yaw = policy.update(desired, t - previous_t)
            points.append(TrajectoryPoint(
                pos=position, vel=velocity, yaw=yaw, t_from_start=t))
            previous_t = t
        return Trajectory(points=points, frame_id="map",
                          stamp_at_start=msg.start_time.to_sec())
