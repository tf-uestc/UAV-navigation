"""Compare physical touchdown, MAVROS disarm, executor completion and mission completion in saved SITL traces."""
import argparse
from bisect import bisect_left
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import tarfile

from audit_batch import GROUND, PAIR, STAMP


def inspect_trace(data):
    trace = sorted(json.loads(data),key=lambda r:r[0])
    land = next((t for t,k,d in trace if k=='flight_command' and d.get('command')=='LAND'),None)
    if land is None:
        return dict(landing_command=None)
    output=dict(land_clock_s=land,trace_sha256=hashlib.sha256(data).hexdigest(), end_wall_s=trace[-1][0]-land)
    states=defaultdict(list)
    latest={}
    odom=[]
    for t,k,d in trace:
        if k=='odom':
            odom.append((t-land,d['stamp'],d['speed']))
        if t<land-.05:
            continue
        if k=='flight':
            state=(d.get('mode'),d.get('reason'),d.get('owns_flight'))
        elif k=='fcu':
            state=(d.get('connected'),d.get('armed'),d.get('mode'))
        elif k=='mission':
            state=(d.get('mission_id'),d.get('phase'),d.get('reason'))
        else:
            continue
        if latest.get(k)!=state:
            states[k].append(dict(relative_wall_s=t-land, state=state))
            latest[k]=state
    disarm=next((v['relative_wall_s'] for v in states['fcu'] if v['relative_wall_s']>=0 and not v['state'][1]),None)
    fault=next((v['relative_wall_s'] for v in states['flight']
                if v['relative_wall_s']>=0 and v['state'][:2]==('FAILED','landing_timeout')),None)
    window=[(w,stamp,s) for w,stamp,s in odom if disarm is not None and fault is not None and disarm<=w<=fault]
    above=[w for w,stamp,s in window if s>.15]
    chronological=[w for w in (disarm, *above, fault) if w is not None]
    output.update(transitions=dict(states), FCU_disarm_s=disarm,executor_failure_s=fault,
                  executor_complete=any(v['state'][0]=='COMPLETE' and not v['state'][2] for v in states['flight']),
                  task_landed=any(v['state'][1]=='LANDED' for v in states['mission']),
                  any_armed_after_disarm=any(v['state'][1] for v in states['fcu'] if disarm is not None and v['relative_wall_s']>disarm),
                  post_land_track_or_plan=sum(1 for t,k,d in trace if t>land and
                     (k=='plan_request' or k=='flight' and d.get('mode')=='TRACK' or k=='flight_command' and d.get('command') in ('START','GOTO','TRACK','PLAN','TAKEOFF'))),
                  trace_contains_timestamped_extended_state=any(k in ('landed','extended_state') for _,k,_ in trace),
                  disarm_to_fault_odom_count=len(window),speed_over_point15=len(above),
                  min_speed_mps=min((s for _,_,s in window),default=None),
                  max_speed_mps=max((s for _,_,s in window),default=None),
                  last_speed_over_point15_after_land_s=above[-1] if above else None,
                  longest_wall_gap_between_motion_updates_including_edges_s=max((b-a for a,b in zip(chronological,chronological[1:])),default=None),
                  _odom=odom)
    return output


def frame_window(data,summary):
    odom=summary['_odom']
    stamps=[s for _,s,_ in odom]
    def wall(s):
        i=bisect_left(stamps,s)
        if not 0<i<len(stamps): return None
        t0,s0,_=odom[i-1];t1,s1,_=odom[i]
        return [t0,t1] if s0<=s<=s1 else None
    start=next(s for w,s,_ in odom if w>=0)
    first=None
    run=None
    last=None
    count=0
    old=0
    frames={}
    for m in STAMP.finditer(data):
        stamp=int(m[1])+int(m[2])*1e-9
        if stamp>=start:
            ground=False
            for a,b in PAIR.findall(data,old,m.start()):
                if b.startswith(b'iris_depth_camera::'):
                    a,b=b,a
                if a.startswith(b'iris_depth_camera::') and b'base_link' in a and b.split(b'::')[0] in GROUND:
                    ground=True
            frames[stamp]=frames.get(stamp,False) or ground
        old=m.end()
    for s,ground in sorted(frames.items()):
        if ground:
            first=s if first is None else first
            run=s if run is None else run
        else:
            run=None
        last=s
        count+=1
    return dict(raw_sha256=hashlib.sha256(data).hexdigest(), first_ground_sim_s=first,
                first_ground_odom_bracket_wall_s=wall(first) if first else None,
                continuous_ground_from_sim_s=run,continuous_ground_from_odom_bracket_wall_s=wall(run) if run else None,
                last_recorded_sim_s=last, frames_after_land_start=count,
                continuous_ground_through_recorder_end_s=last-run if last is not None and run is not None else None)


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('sources',nargs='+',type=Path)
    cli.add_argument('--output',required=True,type=Path)
    cli.add_argument('--scenario-results',type=Path,nargs='+',default=[],help='independent audit_batch JSON files for the same trials')
    args=cli.parse_args()
    planning={}
    for result in args.scenario_results:
        for trial in json.loads(result.read_text(encoding='utf-8'))['trials']:
            label=trial['label']
            if label in planning:
                raise ValueError('Repeated scenario result '+label)
            planning[label]=trial['independent']
    archive_results={}
    for src in args.sources:
        rows={}
        contacts=None
        with tarfile.open(src,'r|*') as archive:
            for m in archive:
                name=m.name.split('/')
                if len(name)!=4 or name[0]!='flights' or not m.isfile():
                    continue
                label=name[1]
                if name[-2:]==['result','trace.json']:
                    rows[label]=inspect_trace(archive.extractfile(m).read())
                if label=='S04-1-unknown' and name[-2:]==['result','physics-contacts.txt']:
                    contacts=archive.extractfile(m).read()
        if contacts is not None and 'S04-1-unknown' in rows:
            rows['S04-1-unknown']['ground_contact']=frame_window(contacts,rows['S04-1-unknown'])
        for row in rows.values():
            row.pop('_odom',None)
        if len(rows)==0:
            raise ValueError('No saved traces in '+str(src))
        archive_results[src.name]=dict(total=len(rows),
            executor_landing_timeouts=[k for k,r in rows.items() if any(x['state'][:2]==('FAILED','landing_timeout') for x in r.get('transitions',{}).get('flight',[]))],
            executor_complete=sum(bool(r.get('executor_complete')) for r in rows.values()),
            mission_landed=sum(bool(r.get('task_landed')) for r in rows.values()),
            missing_task_landed=[k for k,r in rows.items() if not r.get('task_landed')],
            trial_status={k:dict(executor_complete=r.get('executor_complete'),task_landed=r.get('task_landed'),
              trace_sha256=r.get('trace_sha256')) for k,r in rows.items()},
            details={k:r for k,r in rows.items() if r.get('executor_failure_s') is not None})
        if planning:
            decisions={label:bool(planning[label] and r.get('executor_complete') and r.get('task_landed')) for label,r in rows.items()}
            archive_results[src.name]['joined_scenario_and_landing']=dict(passed=sum(decisions.values()),attempted=len(decisions),failed=[k for k,v in decisions.items() if not v])
    if planning and set(planning)!=set().union(*(set(a['trial_status']) for a in archive_results.values())):
        raise ValueError('Scenario and landing inputs refer to different campaigns')
    args.output.write_text(json.dumps(archive_results,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()

