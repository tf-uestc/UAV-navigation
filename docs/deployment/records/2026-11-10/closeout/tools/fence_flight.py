from pathlib import Path
from types import SimpleNamespace
import json
import os
import sys

from gazebo_msgs.srv import GetLinkState
import rospy

from validate_planning_sitl import BODY, PlanningTrial

out = Path(sys.argv[1])
assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11311'
rospy.init_node('closeout_fence_flight', anonymous=True)
rospy.wait_for_service('/gazebo/get_link_state', timeout=8)
assert rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)(BODY, 'world').success
trial = PlanningTrial(SimpleNamespace(mock=False, live=False, yaw=0., target='fence fixture'))
errors = []
try:
    trial.takeoff()
    for _ in range(3):
        invalid = trial.new('outside', waypoint=(6.5, 0., 1.6))
        status = trial.wait_phase(invalid, ('DONE','FAILED'), 12.)
        assert status['phase'] == 'FAILED' and status['reason'] == 'outside_fence', status
        assert trial.flight['mode'] == 'HOLD' and trial.flight['context'] is None
        assert not any(k=='plan_request' and d['mission_id']==invalid for _,k,d in trial.trace)
        trial.check('outside_fence', mission=invalid)
except Exception as exc:
    errors.append(repr(exc))
try:
    trial.stop_and_land()
    assert trial.state and not trial.state.armed
except Exception as exc:
    errors.append('cleanup '+repr(exc))
result = dict(passed=not errors, errors=errors, checks=trial.checks,
              final_armed=trial.state.armed if trial.state else None)
(out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
(out/'trace.json').write_text(json.dumps(trial.trace)+'\n')
print(json.dumps(result), flush=True)
raise SystemExit(bool(errors))
