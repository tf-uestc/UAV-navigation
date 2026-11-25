"""Summarize archived Gazebo/ROS trials without connecting to their ROS master."""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import hashlib
import json
import math
import mmap
from pathlib import Path
import re

PAIR = re.compile(rb'collision1:\s*"([^"]+)"\s+collision2:\s*"([^"]+)"')


def load(path):
    return json.loads(path.read_text())


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + '\n')


def contacts(file):
    counts = Counter()
    with file.open('rb') as handle, mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ) as raw:
        fingerprint = hashlib.sha256(raw).hexdigest()
        for pair in PAIR.finditer(raw):
            names = [x.split(b'::')[0].decode() for x in pair.groups()]
            if names.count('iris_depth_camera') == 2:
                category = 'iris_self'
            elif 'iris_depth_camera' in names:
                other = next(name for name in names if name != 'iris_depth_camera')
                category = 'iris_' + other
            elif set(names) == {'contact_witness','ground_plane'}:
                category = 'witness_ground'
            else:
                category = 'other_' + '/'.join(sorted(names))
            counts[category] += 1
    return dict(file_sha256=fingerprint, observed_pairs=counts)


def flight_window(trace, fence):
    begin = next(t for t,k,d in trace if k=='mission' and 'takeoff' in str(d.get('mission_id'))
                 and d['phase']=='DONE')
    end = next(t for t,k,d in trace if t>begin and k=='mission' and d['phase']=='LANDING')
    points = [d['position'] for t,k,d in trace if begin<=t<=end and k=='odom']
    bounds = [(fence[axis+'_min'],fence[axis+'_max']) for axis in 'xyz']
    outside = sum(any(value<lo or value>hi for value,(lo,hi) in zip(p,bounds)) for p in points)
    return dict(samples=len(points), outside_count=outside, ranges_xyz=[(min(c),max(c)) for c in zip(*points)])


def tracking_errors(trace):
    states = [(t,d) for t,k,d in trace if k=='flight']
    state_times = [t for t,_ in states]
    refs = sorted((d['stamp'],d['position']) for t,k,d in trace if k=='setpoint')
    ref_times = [stamp for stamp,pos in refs]
    errors = []
    track_samples = 0
    for t,k,d in trace:
        if k!='odom':
            continue
        i=bisect_right(state_times,t)-1
        if i<0 or states[i][1]['mode']!='TRACK' or not states[i][1].get('context'):
            continue
        track_samples+=1
        j=bisect_left(ref_times,d['stamp'])
        nearby=refs[max(j-1,0):j+1]
        if nearby:
            stamp,ref=min(nearby,key=lambda row:abs(row[0]-d['stamp']))
            if abs(stamp-d['stamp']) <= .08:
                errors.append(math.dist(ref,d['position']))
    return dict(track_mode_samples=track_samples,aligned_samples=len(errors),
                max_m=max(errors) if errors else None,
                rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None)


def dynamic_trace(trace):
    when, obstacle = next((t,d) for t,k,d in trace if k=='obstacle_inserted')
    mission = next(d['mission_id'] for _,k,d in trace if k=='spline' and d['mission_id'].startswith('dynamic-'))
    odom = [(t,d) for t,k,d in trace if k=='odom']
    events = []
    tracked = set()
    first_hold = None
    warnings = [t for t,k,d in trace if k=='ego_status' and d['mission_id']==mission and d['status']=='EMERGENCY_STOP']
    for t,k,d in trace:
        if k=='flight' and warnings and t>warnings[0] and not first_hold and d['stop_confirmed'] and d['context'] is None and d['mode']=='HOLD':
            first_hold=dict(wall_s=t-when, speed_mps=min(odom,key=lambda row:abs(row[0]-t))[1]['speed'],
                            stop_id=d['stop_id'], reason=d['reason'])
        if k=='flight' and d.get('context') and d['context'][0]==mission and d['mode']=='TRACK':
            tag=(d['context'][1],d['trajectory_id'])
            if tag in tracked:
                continue
            tracked.add(tag)
            details=dict(goal_id=tag[0],traj_id=tag[1])
        elif d.get('mission_id')==mission and k in ('spline','trajectory_ack','mission','ego_status','plan_result'):
            if k=='ego_status' and d['status']=='READY':
                continue
            keys=('goal_id','traj_id','phase','status','reason','accepted','velocity_error_mps','velocity_tolerance_mps','emergency')
            details={key:d[key] for key in keys if key in d}
        else:
            continue
        near=min(odom,key=lambda row:abs(row[0]-t))[1]
        events.append(dict(wall_s=t-when,source=k,nearest_odom_stamp=near['stamp'],
                           measured_speed_mps=near['speed'],position=near['position'],**details))
    return dict(obstacle=obstacle, mission=mission,confirmed_hold_after_warning=first_hold,events=events)


def extract(root, out):
    results = []
    for attempt in sorted((root/'flights').glob('F03-*')):
        source = attempt/'result'
        if not (source/'trace.json').exists() or not (attempt/'independent-planning.json').is_file():
            continue
        report = load(source/'results.json')
        trace = sorted(load(source/'trace.json'),key=lambda item:item[0])
        name=attempt.name
        params=next(c for c in report['checks'] if c['name']=='live_fence_inside_map_and_ceiling')
        geometry={k:v for k,v in params.items() if k!='name'}
        if name=='F03-1-dynamic':
            active = {}
            for line in (attempt/'launch.log').read_text(errors='replace').splitlines():
                m=re.match(r'\s*\* (/(?:ego_planner_node|managed_planner|segment_mission|uav/segment_config)/[^:]+): (.*)$',line)
                if m:
                    active[m[1]]=m[2]
            save(out/'map'/'F03-1-dynamic-active-params.json', dict(rospy_runtime=geometry,roslaunch=active))
        record=dict(id=name, source_commit=(attempt/'commit.txt').read_text().strip(),
                    primary=report['passed'], error=report.get('error'),
                    original_crosscheck=load(attempt/'independent-planning.json')['passed'],
                    original_crosscheck_failed=[k for k,v in load(attempt/'independent-planning.json')['checks'].items() if not v],
                    entire_cruise=flight_window(trace,params['fence']), tracked_setpoint_vs_odom=tracking_errors(trace),
                    metrics=[{k:v for k,v in c.items() if k in ('name','minimum_center_to_box_surface_m',
                                'peak_tracking_error_m','rmse_tracking_m','aligned_samples','odom_samples',
                                'endpoint_speed_mps','segments','mode','reason','executed_trajectories')}
                             for c in report['checks'] if any(k in c for k in
                                ('peak_tracking_error_m','minimum_center_to_box_surface_m'))],
                    raw_contacts=contacts(source/'physics-contacts.txt'))
        if name.endswith(('-dynamic','-blocked')):
            save(out/'scenarios'/(name+'-events.json'),dynamic_trace(trace))
        results.append(record)
    save(out/'scenarios'/'all-planning-attempts.json',results)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('out',type=Path)
    args=parser.parse_args()
    extract(args.root,args.out)
