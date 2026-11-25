"""Exercise the two synthetic fixture callbacks against a saved F03 trace, offline."""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import validate_segments
from validate_planning_sitl import PlanningTrial


class CaptureReplayed(Exception):
    pass


def sample_odom(record):
    message = Odometry()
    message.header.stamp = rospy.Time.from_sec(record['stamp'])
    pose = message.pose.pose
    pose.position.x, pose.position.y, pose.position.z = record['position']
    pose.orientation.z = math.sin(record['yaw'] / 2)
    pose.orientation.w = math.cos(record['yaw'] / 2)
    return message


def replay(trace):
    mission = next(d['mission_id'] for _, k, d in trace if k == 'mission'
                   and d.get('phase') == 'FAILED' and d.get('reason') == 'observation_too_close')
    odometry = [(t, d) for t, k, d in trace if k == 'odom']
    responses = [(t, d) for t, k, d in trace if k == 'detect_result' and d['mission_id'] == mission]
    start_at = next(t for t, k, d in trace if k == 'mission' and d['mission_id'] == mission)
    _, start = min(odometry, key=lambda row: abs(row[0] - start_at))
    trial = SimpleNamespace(odom=sample_odom(start), scenarios={}, flight={'stop_confirmed': True},
                            requests=[], detection=Mock(), record=Mock())
    rows = []

    def new_fixture(_, target):
        trial.scenarios[mission] = {'calls': 0, 'behavior': 'normal', 'target': target}
        for arrival, original in responses:
            stamp = original['capture_stamp']
            _, captured = min(odometry, key=lambda row: abs(row[1]['stamp'] - stamp))
            _, delivered = min(odometry, key=lambda row: abs(row[0] - arrival))
            assert abs(captured['stamp'] - stamp) < .001
            trial.odom = sample_odom(captured)
            request = {'mission_id': mission, 'request_id': original['request_id'],
                       'target': {'text': original['target_text']}}
            validate_segments.Trial.detect_cb(trial, String(data=json.dumps(request)))
            result = json.loads(trial.detection.publish.call_args[0][0].data)
            assert abs(result['capture_stamp'] - stamp) < .001
            raw_distance = math.dist(result['point'][:2], captured['position'][:2])
            assert abs(raw_distance - 2.4) < 1.e-5
            corrected = math.dist(result['point'][:2], delivered['position'][:2])
            assert abs(corrected - 2.4) <= .12
            rows.append({'stamp': stamp, 'old_distance_at_delivery_m':
                         math.dist(original['point'][:2], delivered['position'][:2]),
                         'new_distance_at_capture_m': raw_distance,
                         'new_distance_at_delivery_m': corrected})
        raise CaptureReplayed

    trial.new = new_fixture
    def sync_thread(target, daemon):
        return SimpleNamespace(start=target)

    with patch.object(rospy, 'get_param', return_value=2.4), \
            patch.object(rospy, 'is_shutdown', return_value=False), \
            patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(90)), \
            patch.object(validate_segments.time, 'sleep'), \
            patch.object(validate_segments.threading, 'Thread', sync_thread):
        try:
            PlanningTrial.face_route(trial, (4.2, -1.35, 1.6))
        except CaptureReplayed:
            pass
    assert len(rows) >= 2
    assert any(row['old_distance_at_delivery_m'] < 2.4 - .12 for row in rows)
    assert trial.record.call_count == trial.detection.publish.call_count == len(rows)
    return {'passed': True, 'mission': mission, 'observations': rows}


if __name__ == '__main__':
    print(json.dumps(replay(json.loads(Path(sys.argv[1]).read_text())), indent=2))
