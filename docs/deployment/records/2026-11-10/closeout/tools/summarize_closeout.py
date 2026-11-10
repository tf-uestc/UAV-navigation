from pathlib import Path
import json
import math
from statistics import median
import sys

root = Path(sys.argv[1])


def load(name):
    return json.loads((root/name).read_text())


def timing(name):
    trace = load('regression-'+name+'/trace.json')
    start = next((t for t,k,d in trace if k=='mission' and d['phase']=='TAKEOFF'),None)
    done = next((t for t,k,d in trace if start is not None and t>=start and k=='mission' and d['phase']=='DONE'),None)
    land = next((t for t,k,d in trace if k=='mission' and d['phase']=='LANDING'),None)
    disarmed = next((t for t,k,d in trace if land is not None and t>=land and k=='fcu' and d['armed'] is False),None)
    return dict(start=bool(start is not None),
                takeoff_intent_to_done_s=done-start if done is not None else None,
                done_to_land_intent_s=land-done if done is not None and land is not None else None,
                takeoff_intent_to_disarm_s=disarmed-start if disarmed is not None and start is not None else None)


def stat(samples, key):
    values = [s[key] for s in samples]
    if not values:
        return None
    return dict(minimum=min(values), median=median(values), maximum=max(values))


def rgbd(path):
    r = json.loads(path.read_text())
    samples = r.pop('samples')
    rejected = sum(r.get('rejected',{}).values())
    completed = len(samples)+rejected
    r.update(dict(speed_mps=stat(samples,'speed_mps'), heading_deg=stat(samples,'heading_deg'),
                  target_distance_m=stat(samples,'target_distance_m'),
                  truth_time_offset_s=stat(samples,'truth_time_offset_s'),
                  gazebo_radial_p95_m=stat(samples,'gazebo_surface_error_p95_m'),
                  rejected_fraction_of_classified=rejected/completed if completed else None))
    if samples and 'yaw_alignment_delta_deg' in samples[0]:
        r['yaw_alignment_delta_deg'] = stat(samples, 'yaw_alignment_delta_deg')
    return r


report = dict(
    simulator_static=load('restore/restored-static/validation.json'),
    dynamic=load('dynamic-analysis.json'),
    motion={p.parent.name:rgbd(p) for p in sorted(root.glob('motion-yaw*/motion-rgbd.json'))},
    alignment={p.parent.name:rgbd(p) for p in sorted(root.glob('alignment-yaw*/motion-rgbd.json'))},
    alignment_initial={p.parent.name:{k:v for k,v in json.loads(p.read_text()).items() if k!='samples'}
                       for p in sorted(root.glob('alignment-yaw*/alignment-ready.json'))},
    planner_only=load('unknown-scope/results.json'),
    regression={name:dict(load('regression-'+name+'/results.json'),timing=timing(name))
                for name in ('static','fence','replay','live') if (root/('regression-'+name+'/results.json')).exists()})
(root/'closeout-summary.json').write_text(json.dumps(report,indent=2)+'\n')
for group in ('motion','alignment','alignment_initial'):
    for name,data in report[group].items():
        print(group,name,json.dumps(data,sort_keys=True))
for name,data in report['regression'].items():
    print('regression',name,data['passed'],data['timing'],data.get('error',data.get('errors')),
          [(item['name'],{k:v for k,v in item.items() if k!='name'}) for item in data['checks'][-6:]])
