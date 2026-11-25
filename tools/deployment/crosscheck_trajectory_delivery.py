"""Account for every recorded native spline in a simulated dynamic/blocked mission."""
import json
import sys
from pathlib import Path


def key(record):
    return (record['planner_session_id'], record['mission_id'], record['goal_id'], record['traj_id'])


def check(trace):
    trace = sorted(trace, key=lambda row: row[0])
    native = [(t, d) for t, k, d in trace if k == 'native_spline' and d['mission_id'].startswith('dynamic-')]
    forwarded = {key(d) for _, k, d in trace if k == 'forwarded_spline'}
    acks = {key(d): d for _, k, d in trace if k == 'trajectory_ack'}
    rejected = {key(d): d for _, k, d in trace if k == 'gateway_disposition' and d.get('accepted') is False}
    critical = {d['mission_id'] for _, k, d in trace if k == 'ego_status'
                and d['status']=='EMERGENCY_STOP' and d['reason']=='collision'}
    dispositions = []
    for t, packet in native:
        identity = key(packet)
        a = acks.get(identity)
        g = rejected.get(identity)
        accounted = ((identity in forwarded and a is not None and not g)
                     or (identity not in forwarded and a is None and bool(g) and packet['emergency']))
        dispositions.append(dict(native=packet, gateway_forwarded=identity in forwarded,
                                executor_ack=a, gateway_rejection=g, accounted=accounted))
    has_emergency = any(d['emergency'] and d['mission_id'] in critical for _, d in native)
    return dict(passed=bool(native) and has_emergency and all(row['accounted'] for row in dispositions),
                collision_warning_missions=sorted(critical), native_emergency_recorded=has_emergency,
                dispositions=dispositions)


if __name__ == '__main__':
    report = check(json.loads((Path(sys.argv[1])/'result'/'trace.json').read_text()))
    print(json.dumps(report, indent=2))
    raise SystemExit(not report['passed'])
