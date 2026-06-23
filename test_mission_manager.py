"""Lightweight mission_manager tests with mocked ROS modules."""
import json
import sys
import types


# ---------------------------------------------------------------------------
# Mock rospy
# ---------------------------------------------------------------------------
published = {}


class MockPublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []
        published[topic] = self

    def publish(self, msg):
        self.messages.append(msg)


class MockTime:
    _now = 0.0

    def __init__(self, secs=0, nsecs=0):
        self.value = float(secs) + float(nsecs) * 1e-9

    def to_sec(self):
        return self.value

    def __sub__(self, other):
        return MockDuration(self.value - other.value)

    @staticmethod
    def now():
        return MockTime(MockTime._now)


class MockDuration:
    def __init__(self, secs):
        self.value = float(secs)

    def to_sec(self):
        return self.value


rospy_mock = types.ModuleType("rospy")
rospy_mock.init_node = lambda *a, **k: None
rospy_mock.get_param = lambda k, d=None: d
rospy_mock.Subscriber = lambda *a, **k: None
rospy_mock.Publisher = lambda topic, *a, **k: MockPublisher(topic)
rospy_mock.Time = MockTime
rospy_mock.Duration = MockDuration
rospy_mock.loginfo = lambda *a, **k: None
rospy_mock.logwarn = lambda *a, **k: None
rospy_mock.logerr = lambda *a, **k: None
rospy_mock.loginfo_throttle = lambda *a, **k: None
rospy_mock.logwarn_throttle = lambda *a, **k: None
rospy_mock.is_shutdown = lambda: False
rospy_mock.Rate = lambda hz: type("R", (), {"sleep": lambda self: None})()
rospy_mock.Timer = lambda period, cb, oneshot=False: type("T", (), {})()
rospy_mock.sleep = lambda duration: None
sys.modules["rospy"] = rospy_mock


# ---------------------------------------------------------------------------
# Mock ROS msg modules
# ---------------------------------------------------------------------------
class Header:
    def __init__(self, seq=0, stamp=None, frame_id=""):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id


class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Pose:
    def __init__(self, position=None, orientation=None):
        self.position = position or Point()
        self.orientation = orientation or Quaternion()


class PoseStamped:
    def __init__(self, header=None, pose=None):
        self.header = header or Header()
        self.pose = pose or Pose()


class PointStamped:
    def __init__(self, header=None, point=None):
        self.header = header or Header()
        self.point = point or Point()


class Path:
    def __init__(self, header=None, poses=None):
        self.header = header or Header()
        self.poses = poses if poses is not None else []


class String:
    def __init__(self, data=None):
        self.data = data


geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs.msg.PointStamped = PointStamped
geometry_msgs.msg.Point = Point
geometry_msgs.msg.PoseStamped = PoseStamped
geometry_msgs.msg.Quaternion = Quaternion
geometry_msgs.msg.Pose = Pose
sys.modules["geometry_msgs"] = geometry_msgs
sys.modules["geometry_msgs.msg"] = geometry_msgs.msg

nav_msgs = types.ModuleType("nav_msgs")
nav_msgs.msg = types.ModuleType("nav_msgs.msg")
nav_msgs.msg.Path = Path
sys.modules["nav_msgs"] = nav_msgs
sys.modules["nav_msgs.msg"] = nav_msgs.msg

std_msgs = types.ModuleType("std_msgs")
std_msgs.msg = types.ModuleType("std_msgs.msg")
std_msgs.msg.String = String
std_msgs.msg.Header = Header
sys.modules["std_msgs"] = std_msgs
sys.modules["std_msgs.msg"] = std_msgs.msg

ego_planner = types.ModuleType("ego_planner")
ego_planner.msg = types.ModuleType("ego_planner.msg")
ego_planner.msg.Bspline = type("Bspline", (), {})
sys.modules["ego_planner"] = ego_planner
sys.modules["ego_planner.msg"] = ego_planner.msg

tf = types.ModuleType("tf")
tf.transformations = types.ModuleType("tf.transformations")
tf.transformations.quaternion_from_euler = lambda r, p, y: (0.0, 0.0, 0.0, 1.0)
sys.modules["tf"] = tf
sys.modules["tf.transformations"] = tf.transformations


# ---------------------------------------------------------------------------
# Actual tests
# ---------------------------------------------------------------------------
sys.path.insert(0, "/home/tf/毕业设计/ws/src/uav_vln_bringup/scripts")
from mission_manager import (  # noqa: E402
    APPROACH_FLYING,
    APPROACH_WAIT_PLAN,
    APPROACH_WAIT_VLM,
    MissionManager,
    STATE_APPROACHING,
    STATE_IDLE,
    STATE_SEARCHING,
)


def make_target(seq=0, x=1.0, y=2.0, z=3.0):
    return PointStamped(
        header=Header(seq=seq, stamp=MockTime.now(), frame_id="map"),
        point=Point(x, y, z),
    )


mm = MissionManager()
assert mm._state == STATE_IDLE, f"Expected IDLE, got {mm._state}"

# SEARCHING target is accepted without request_id and forwarded as approach_target.
mm._state = STATE_SEARCHING
mm._target_text = "red target"
mm._current_pose = PoseStamped(pose=Pose(Point(0.0, 0.0, 2.0), Quaternion()))
mm._on_target_world(make_target(seq=0))
assert mm._state == STATE_APPROACHING
assert mm._approach_phase == APPROACH_WAIT_PLAN
assert published["/uav/approach_target"].messages, "initial approach target not forwarded"

# Old planner OK must not leave WAIT_PLAN.
mm._on_planner_status(String("OK"))
assert mm._approach_phase == APPROACH_WAIT_PLAN

# New non-empty trajectory is the condition to fly again.
mm._on_trajectory(Path(poses=[PoseStamped()]))
assert mm._approach_phase == APPROACH_FLYING

# During WAIT_VLM, stale request ids are ignored.
mm._approach_phase = APPROACH_WAIT_VLM
mm._pending_detection_request_id = 42
mm._detection_in_flight = True
before_count = len(published["/uav/approach_target"].messages)
mm._on_target_world(make_target(seq=41, x=9.0))
assert len(published["/uav/approach_target"].messages) == before_count
assert mm._pending_detection_request_id == 42

# Matching request ids are accepted and forwarded.
mm._on_target_world(make_target(seq=42, x=4.0))
assert mm._approach_phase == APPROACH_WAIT_PLAN
assert mm._pending_detection_request_id is None
assert len(published["/uav/approach_target"].messages) == before_count + 1

# Detector BUSY is handled as an immediate miss/timeout path.
mm._approach_phase = APPROACH_WAIT_VLM
mm._pending_detection_request_id = 99
mm._detection_in_flight = True
miss_before = mm._miss_count
mm._on_detector_status(String(json.dumps({"request_id": 99, "status": "BUSY"})))
assert mm._miss_count == miss_before + 1
assert mm._pending_detection_request_id is None

print("PASS: mission_manager approach contracts OK")
