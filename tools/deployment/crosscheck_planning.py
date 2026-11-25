#!/usr/bin/env python3
"""Independently grade a saved U08 run using the time-ordered ROS and Gazebo records."""
import argparse
import json
import math
from pathlib import Path

PREFIXES = dict(straight=('straight-',), outside=('outside-',), long=('beyond_planning_horizon-',),
                static=('occupied-', 'bypass-'), dynamic=('dynamic-',), blocked=('dynamic-',))


def grade(attempt, scenario):
    folder = attempt / 'result'
    trace = json.loads((folder/'trace.json').read_text())
    checks = {}
    details = []
    evidence = json.loads((folder/'physics-contacts.json').read_text())
    checks['contact_stream_observed'] = (evidence['positive_control_contacts'] > 0
        and evidence['physics_messages'] >= 100 and evidence['passed']
        and not evidence['iris_obstacle_contacts'] and 0 <= evidence['end_stamp']-evidence['last_message_stamp'] < 1.)
    fcu = [d for _,k,d in trace if k=='fcu']
    checks['armed_then_disarmed'] = any(d['armed'] for d in fcu) and not fcu[-1]['armed']
    phases = [(t,d) for t,k,d in trace if k=='mission' and
              str(d.get('mission_id')).startswith(PREFIXES[scenario])]
    missions = sorted({d['mission_id'] for _,d in phases})
    checks['all_requested_missions_present'] = all(any(m.startswith(p) for m in missions) for p in PREFIXES[scenario])
    for mission in missions:
        subset = [(t,d) for t,d in phases if d['mission_id']==mission]
        terminal = next(((t,d) for t,d in subset if d['phase'] in ('DONE','FAILED')),None)
        checks[mission+'/terminal'] = terminal is not None
        if not terminal:
            continue
        stamp,state = terminal
        if mission.startswith(('occupied-','outside-')) or scenario=='blocked':
            checks[mission+'/stopped'] = state['phase']=='FAILED'
            holds=[d for t,k,d in trace if k=='flight' and stamp-.3<=t<=stamp+.7 and d['mode']=='HOLD'
                   and d['context'] is None and d['stop_confirmed']]
            checks[mission+'/measured_hold']=bool(holds)
            if mission.startswith('outside-'):
                checks[mission+'/outside'] = state['reason']=='outside_fence' and not any(
                    k=='plan_request' and d['mission_id']==mission for t,k,d in trace)
        else:
            checks[mission+'/reached'] = state['phase']=='DONE'
            odom = [(t,d) for t,k,d in trace if k=='odom' and subset[0][0]-.3<=t<=stamp+.2]
            checks[mission+'/fence'] = bool(odom) and all(-6.<=d['position'][0]<=6. and -4.<=d['position'][1]<=4.
                    and 1.1<=d['position'][2]<=2.35 for _,d in odom)
            if odom and state.get('final_goal'):
                _,closest = min(odom,key=lambda row:abs(row[0]-stamp))
                error = math.dist(closest['position'],state['final_goal'])
                checks[mission+'/physical_endpoint'] = error<=.23 and closest['speed']<=.15
                details.append(dict(mission=mission,terminal_reason=state['reason'],
                                    distance_to_goal_m=error,speed_mps=closest['speed']))
            else:
                checks[mission+'/physical_endpoint'] = False
            if scenario=='long' and odom:
                checks[mission+'/beyond_horizon'] = (math.dist(odom[0][1]['position'],state['final_goal'])>7.5
                                                      and state['segment_count']>=5)
        ids = {(d['goal_id'],d['traj_id']) for _,k,d in trace if k=='spline' and d['mission_id']==mission}
        decisions = [d for _,k,d in trace if k=='trajectory_ack' and d['mission_id']==mission]
        checks[mission+'/acknowledgements'] = ids <= {(d['goal_id'],d['traj_id']) for d in decisions}
        executed = {(d['context'][1],d['trajectory_id']) for _,k,d in trace if k=='flight' and
                    d.get('mode')=='TRACK' and d['context'] and d['context'][0]==mission}
        checks[mission+'/only_accepted_executed'] = executed <= {
            (d['goal_id'],d['traj_id']) for d in decisions if d['accepted']}
        if state['phase']=='DONE':
            checks[mission+'/tracked'] = bool(executed)
    if scenario in ('static','dynamic','blocked'):
        event = 'world_obstacle_track' if scenario=='static' else 'dynamic_box_world_track'
        sample = next((d for _,k,d in trace if k==event),None)
        fixture = sample if scenario=='static' else next((d for _,k,d in trace if k=='obstacle_inserted'),None)
        checks['world_samples_present']=bool(sample and fixture and sample['samples'])
        if checks['world_samples_present']:
            cx,cy=fixture['center']
            sx,sy=fixture['size'][:2]
            world=sample['samples']
            clearance=min(math.hypot(max(abs(x-cx)-sx/2,0.),max(abs(y-cy)-sy/2,0.)) for x,y,z in world)
            details.append(dict(samples=len(world),center_to_obstacle_surface_m=clearance))
            checks['clearance']=clearance>.24
            checks['world_outcome']=(max(p[0] for p in world)<cx-sx/2 if scenario=='blocked' else world[-1][0]>cx+.35)
    return dict(passed=all(checks.values()),scenario=scenario,checks=checks,details=details)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario',choices=PREFIXES)
    parser.add_argument('attempt',type=Path)
    args=parser.parse_args()
    report=grade(args.attempt,args.scenario)
    (args.attempt/'independent-planning.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
    raise SystemExit(not report['passed'])
