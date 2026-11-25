"""Compare the two hold clocks in a pre-fix flight archive, read-only."""
import argparse
import json
import math
from pathlib import Path


def replay(case):
    trace = json.loads((case / 'result/trace.json').read_text())
    samples = json.loads((case / 'clearance.json').read_text())['samples']
    odom = [(time, d) for time, kind, d in trace if kind == 'odom']
    done = next((time for time, kind, d in trace if kind == 'mission' and d['phase'] == 'DONE'), None)
    if done is None:
        return {'case': case.name, 'reached': False}
    land = next(time for time, kind, d in trace if time > done and kind == 'flight_command' and d.get('command') == 'LAND')
    def stamp(wall):
        return min(odom, key=lambda row: abs(row[0]-wall))[1]['stamp']
    first, last = stamp(done), stamp(land)
    old = [p for p in samples if first+.5 <= p['stamp'] <= first+9.5]
    pre = [p for p in samples if first <= p['stamp'] < last]
    def nearest(point):
        return min((d for _, d in odom), key=lambda d: abs(d['stamp']-point['stamp']))
    return dict(case=case.name, reached=True, pre_land_sim_seconds=last-first,
                legacy_window_samples_after_land=sum(p['stamp']>=last for p in old),
                world_dz_old_window_m=old[-1]['world_xyz'][2]-old[0]['world_xyz'][2],
                matched_odom_dz_old_window_m=nearest(old[-1])['position'][2]-nearest(old[0])['position'][2],
                pre_land_world_drift_m=max(math.dist(p['world_xyz'],pre[0]['world_xyz']) for p in pre))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    print(json.dumps([replay(p) for p in sorted(args.archive.glob('*-a[12]'))],indent=2))
