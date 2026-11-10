from pathlib import Path
from types import SimpleNamespace
import json
import math
import os
import sys

from gazebo_msgs.srv import GetLinkState
import rospy

from validate_planning_sitl import BODY, PlanningTrial, decode

out = Path(sys.argv[1])
yaw = float(sys.argv[2])
assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11311'
rospy.init_node('closeout_motion_flight', anonymous=True)
rospy.wait_for_service('/gazebo/get_link_state', timeout=8)
assert rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)(BODY, 'world').success
trial = PlanningTrial(SimpleNamespace(mock=False, live=False, yaw=yaw, target='motion RGB-D check'))
errors = []
try:
    trial.takeoff()
    point, actual_yaw = decode(trial.odom.pose.pose)
    goal = (point[0]+1.25*math.cos(actual_yaw), point[1]+1.25*math.sin(actual_yaw), 1.6)
    trial.move('moving_rgbd_route', goal, 30)
except Exception as exc:
    errors.append(repr(exc))
try:
    trial.stop_and_land()
    assert trial.state and not trial.state.armed
except Exception as exc:
    errors.append('cleanup '+repr(exc))
result = dict(passed=not errors, errors=errors, checks=trial.checks,
              final_armed=trial.state.armed if trial.state else None)
(out/'motion-flight.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result), flush=True)
raise SystemExit(bool(errors))
