"""Reconcile a physics stream that ends between the two odometry samples around disarm."""
import argparse
from bisect import bisect_left
import hashlib
import json
import mmap
from pathlib import Path

from verify_planning import PAIR, STAMP


def read_json(path):
    return json.loads(path.read_text())


def inspect(folder, original_audit, deployment):
    result = folder / 'result'
    primary = read_json(result / 'results.json')
    contact = read_json(result / 'physics-contacts.json')
    trace = read_json(result / 'trace.json')
    flight_exit = (folder / 'flight.exit').read_text().strip()
    issues = [key for key, ok in original_audit['checks'].items() if not ok]
    physics_issues = [key for key, ok in original_audit['physics_coverage']['checks'].items() if not ok]

    states = sorted((t, d['armed']) for t, k, d in trace if k == 'fcu')
    last_armed = max(t for t, armed in states if armed)
    disarm = next(t for t, armed in states if t > last_armed and not armed)
    odom = sorted((t, d['stamp']) for t, k, d in trace if k == 'odom')
    index = bisect_left([t for t, _ in odom], disarm)
    before_disarm, after_disarm = odom[index-1][1], odom[index][1]
    complete = [t for t, k, d in trace if k == 'flight' and t >= disarm
                and d['mode'] == 'COMPLETE' and d['owns_flight'] is False]

    # The recorder obtains this Gazebo clock observation after stop_and_land
    # has verified landed and disarmed. Check that order against the recorded
    # runtime version rather than using an inferred next odometry sample.
    validator = (deployment / 'validate_planning_sitl.py').read_bytes()
    trial = (deployment / 'validate_segments.py').read_bytes()
    observer = (deployment / 'observe_physics_contacts.py').read_bytes()
    main = validator[validator.index(b'def main():'):]
    land_body = trial[trial.index(b'    def stop_and_land(self):'):trial.index(b'    def initial(self):')]
    finish_body = observer[observer.index(b' def finish(self):'):]
    source_order = (main.index(b'trial.stop_and_land()') < main.index(b'not trial.state.armed')
                    < main.index(b'report=contacts.finish()')
                    and b'not self.state.armed' in land_body
                    and b'self.landed.landed_state==1' in land_body
                    and finish_body.index(b'finished=rospy.Time.now().to_sec()')
                    < finish_body.index(b'send_signal(signal.SIGINT)')
                    and b'end_stamp=finished' in finish_body)
    stop = contact['end_stamp']
    window = []
    previous_end = 0
    with (result / 'physics-contacts.txt').open('rb') as handle, mmap.mmap(
            handle.fileno(), 0, access=mmap.ACCESS_READ) as raw:
        for envelope in STAMP.finditer(raw):
            sec, nsec = map(int, envelope.groups())
            stamp = sec + nsec * 1.e-9
            if stop-1.-1.e-9 <= stamp <= stop+1.e-9:
                body_ground = False
                for left, right in PAIR.findall(raw, previous_end, envelope.start()):
                    if b'iris_depth_camera::' in right:
                        left, right = right, left
                    body_ground |= (b'iris_depth_camera::' in left and b'base_link' in left
                                    and right.split(b'::')[0] in (b'ground_plane', b'landing_pad'))
                window.append((stamp, body_ground))
            previous_end = envelope.end()
    window.sort()
    checks = dict(primary_landed_and_passed=flight_exit == '0' and primary['passed']
                  and primary['final_armed'] is False and bool(complete) and not states[-1][1],
                  prior_failure_only_conservative_end=(issues == ['contact_stream_observed']
                  and physics_issues == ['brackets_entire_flight']
                  and original_audit['physics_coverage']['raw_physics_interval'][0]
                  <= original_audit['physics_coverage']['armed_flight_conservative_interval'][0]),
                  post_landing_stop_is_between_odom_samples=source_order
                  and before_disarm <= stop < after_disarm,
                  raw_physics_reaches_post_disarm_stop=abs(stop-contact['last_message_stamp']) < 1.e-9,
                  direct_ground_contact_in_every_final_frame=len(window) >= 100
                  and window[0][0] <= stop-1.+1.e-9 and abs(window[-1][0]-stop) < 1.e-9
                  and all(grounded for _, grounded in window)
                  and max(b[0]-a[0] for a, b in zip(window, window[1:])) <= .008001)
    return dict(passed=all(checks.values()), checks=checks, disarm_bracketing_odom=[before_disarm,after_disarm],
                directly_recorded_stop_after_landing=stop, contact_frames_last_second=len(window),
                frames_with_body_on_ground=sum(grounded for _,grounded in window),
                sources={name: hashlib.sha256(source).hexdigest() for name,source in
                [('validate_planning_sitl.py',validator),('validate_segments.py',trial),
                 ('observe_physics_contacts.py',observer)]})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('attempt', type=Path)
    parser.add_argument('audit', type=Path)
    parser.add_argument('deployment', type=Path)
    args = parser.parse_args()
    report = inspect(args.attempt, read_json(args.audit), args.deployment)
    print(json.dumps(report, indent=2))
    raise SystemExit(not report['passed'])
