#!/usr/bin/python3
"""Independently verify saved dynamic flight traces without commanding a simulator."""
import argparse,json,math
from pathlib import Path
def validate(folder):
 t=json.loads((folder/'trace.json').read_text())
 original=json.loads((folder/'results.json').read_text())
 contacts=json.loads((folder/'physics-contacts.json').read_text())
 inserted=next((tm,d) for tm,k,d in t if k=='obstacle_inserted')
 track=next(d for tm,k,d in t if k=='dynamic_box_world_track')['samples']
 phases=[(tm,d) for tm,k,d in t if k=='mission' and str(d.get('mission_id','')).startswith('dynamic-')]
 assert phases
 mission=phases[0][1]['mission_id']
 done=next((tm,d) for tm,d in phases if d['phase']=='DONE')
 terminal=next((tm,d) for tm,d in phases if d['phase'] in ('DONE','FAILED'))
 assert terminal[1]['phase']=='DONE',terminal
 center=inserted[1]['center'];sx,sy,_=inserted[1]['size']
 minimum=min(math.hypot(max(0.,abs(x-center[0])-sx/2),max(0.,abs(y-center[1])-sy/2)) for x,y,z in track)
 assert minimum>.24 and track[-1][0]>center[0]+.35
 od=[d for tm,k,d in t if k=='odom' and phases[0][0]<=tm<=done[0]]
 assert od and all(-6<=d['position'][0]<=6 and -4<=d['position'][1]<=4 and 1.1<=d['position'][2]<=2.35 for d in od)
 statuses=[(tm,d) for tm,k,d in t if k=='ego_status' and tm>=inserted[0] and d['mission_id']==mission]
 guards={'occupied_trajectory','unknown_trajectory','outside_known_map'}
 guard=next(((tm,d) for tm,d in statuses if d['status']=='FAILED' and d['reason'] in guards),None)
 emergency=next(((tm,d) for tm,d in statuses if d['status']=='EMERGENCY_STOP' and d['reason']=='collision'),None)
 stopped=guard or emergency
 assert stopped is not None,'no recorded safety stop after obstacle insertion'
 reasons={'planner_stop'}|guards
 stop_at=next(tm for tm,d in phases if tm>=stopped[0]-.3 and d['phase']=='SETTLE' and d['reason'] in reasons)
 resumed=next(tm for tm,d in phases if tm>stop_at and d['phase']=='TRACK' and d['goal_id']>1 and d.get('detour_active'))
 assert any(k=='flight' and stop_at<=tm<resumed and d['mode']=='HOLD' and d['context'] is None and d['stop_confirmed'] for tm,k,d in t)
 assert any(stop_at<tm<resumed and d['phase']=='ALIGN_MAP' for tm,d in phases)
 assert any(stop_at<tm<resumed and d['phase']=='OBSERVE_MAP' for tm,d in phases)
 active=[d for tm,k,d in t if tm>=resumed and k=='flight' and d['mode']=='TRACK']
 assert active and all(d['context'] and d['context'][0]==mission and d['context'][1]>1 for d in active)
 assert contacts['passed'] and contacts['positive_control_contacts']>0 and not contacts['iris_obstacle_contacts']
 assert original['final_armed'] is False
 report=dict(passed=True,mission=mission,minimum_center_to_obstacle_surface_m=minimum,
  executed_goal_ids=sorted({d['context'][1] for d in active}),elapsed_after_insertion_s=done[0]-inserted[0],
  stop_reason=stopped[1]['reason'],confirmed_stop_before_new_authorization=True,old_goal_never_revived=True,
  map_observed_before_detour=True,physics_positive_control_contacts=contacts['positive_control_contacts'],
  iris_obstacle_contacts=0,final_armed=False,original_harness_passed=original['passed'],
  original_harness_error=original.get('error'))
 (folder/'independent-crosscheck.json').write_text(json.dumps(report,indent=2)+'\n')
 return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folders',nargs='+',type=Path);a=p.parse_args()
 for folder in a.folders:print(json.dumps(validate(folder)),flush=True)
