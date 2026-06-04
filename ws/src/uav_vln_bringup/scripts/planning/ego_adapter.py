"""planning/ego_adapter.py — EgoPlannerAdapter wrapping EGO-Planner.

Publishes goals to /move_base_simple/goal, subscribes to EGO's
/planning/bspline output, and converts it to the Trajectory format
used by planner_node.
"""

import time as _time
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
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
            rospy.AnyMsg,
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
        self._make_goal(goal)

        self._last_traj = None
        deadline = rospy.Time.now() + rospy.Duration(self._plan_timeout)

        while not rospy.is_shutdown() and self._last_traj is None:
            if rospy.Time.now() > deadline:
                rospy.logerr("[ego_adapter] Plan timed out after %.1f s",
                             self._plan_timeout)
                return None
            rospy.sleep(0.05)

        return self._last_traj

    def shutdown(self) -> None:
        if HAS_BSPLINE_MSG and self._bspline_sub is not None:
            self._bspline_sub.unregister()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _on_odom(self, _msg) -> None:
        self._odom_received = True

    def _on_bspline(self, msg) -> None:
        self._last_traj = self._bspline_to_trajectory(msg)

    def _make_goal(self, goal: Point) -> None:
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
    def _bspline_to_trajectory(msg) -> Trajectory:
        """Convert ego_planner/Bspline to Trajectory.

        Uses linear interpolation of pos_pts as a fallback (the C++
        UniformBspline bindings are not available in Python).
        """
        pts = msg.pos_pts  # list of geometry_msgs/Point

        if not pts:
            return Trajectory(frame_id="map")

        # Linear interpolation with fixed 0.1 s spacing
        total_pts = max(2, len(pts))
        dt = 0.1
        total_time = (total_pts - 1) * dt

        trajectory_pts: list[TrajectoryPoint] = []
        for i in range(total_pts):
            t = i * dt
            trajectory_pts.append(TrajectoryPoint(
                pos=(pts[i].x, pts[i].y, pts[i].z),
                t_from_start=t,
            ))

        return Trajectory(
            points=trajectory_pts,
            frame_id="map",
        )
