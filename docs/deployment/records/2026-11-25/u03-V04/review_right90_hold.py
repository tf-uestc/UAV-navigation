#!/usr/bin/env python3
"""Decompose the true endpoint and pre-land hold using only archived flight data."""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def read(p): return json.loads(p.read_text(encoding='utf-8-sig'))


def near(rows,t,key='stamp'):
    return min(rows,key=lambda r:abs(r[key]-t))


def xyz(T,point):
    return (T@[*point,1.])[:3]


def band_runs(samples,nominal,tolerance):
    """Use recorded sample times. Return crossings under piecewise linear interpolation."""
    def error(s):return math.dist(s['world_xyz'][:2],s['target_xyz'][:2])-nominal
    spans=[]
    for (a,b) in zip(samples,samples[1:]):
        t0,t1=a['stamp'],b['stamp']
        e0,e1=error(a),error(b)
        for sign in (-1,1):
            v0,v1=sign*e0-tolerance,sign*e1-tolerance
            if max(v0,v1)<=0: continue
            start,end=t0,t1
            if min(v0,v1)<=0:
                cross=t0+(t1-t0)*v0/(v0-v1)
                if v0>0:end=cross
                else:start=cross
            if spans and spans[-1]['sign']==sign and abs(spans[-1]['end_s']-start)<1e-7:
                spans[-1]['end_s']=end
            else:
                spans.append(dict(sign=sign,start_s=start,end_s=end))
    return spans


def audit(case):
    nominal=2.4
    truth=read(case/'clearance.json')['samples']
    tr=read(case/'result/trace.json')
    result=read(case/'result/results.json')
    checks={s['name']:s for s in result['checks']}
    mid=checks['attempt_observations']['mission_id']
    moving=read(case/'moving.json')
    T=np.linalg.inv(moving['ground_alignment_transform'])
    od=[r for _,k,r in tr if k=='odom']
    sp=[r for _,k,r in tr if k=='setpoint']
    phases=[r for _,k,r in tr if k=='mission' and r.get('mission_id')==mid]
    done=next(r for r in phases if r['phase']=='DONE')
    t=done['observed_sim']
    requests=[r for _,k,r in tr if k=='search_capture' and r.get('mission_id')==mid]
    replies=[r for _,k,r in tr if k=='detect_result' and r.get('mission_id')==mid]
    reply=replies[-1]
    assert requests[-1]['request_id']==reply['request_id'] and reply['status']=='SUCCEEDED'
    cap=reply['capture_stamp']
    true_done=near(truth,t)
    center=np.asarray(true_done['target_xyz'])
    body_world=np.asarray(true_done['world_xyz'])
    pmap=np.asarray(done['final_goal'])
    tmap=np.asarray(reply['point'])
    ebody,etarget=xyz(T,pmap),xyz(T,tmap)
    def rng(q,ref):return float(np.linalg.norm(q[:2]-ref[:2]))
    prod=rng(pmap,tmap)
    rotated=rng(ebody,etarget)
    estimate_to_center=rng(ebody,center)
    actual=rng(body_world,center)
    odo,cmd=near(od,t),near(sp,t)
    cs=checks['observed_and_held'];start,end=cs['start_sim'],cs['end_sim']
    window=[s for s in truth if start<=s['stamp']<=end]
    setpoints=[s for s in sp if start<=s['stamp']<=end]
    dd=lambda s:rng(np.asarray(s['world_xyz']),np.asarray(s['target_xyz']))
    lo,hi=min(window,key=dd),max(window,key=dd)
    radial=body_world[:2]-center[:2];radial=radial/np.linalg.norm(radial)
    vt=np.array([-radial[1],radial[0]])
    hi_delta=np.array(hi['world_xyz'])-np.array(window[0]['world_xyz'])
    out=[s for s in window if abs(dd(s)-nominal)>.20]
    phys=near(truth,cap)
    odcap=near(od,cap)
    def world(s):return dict(sim_s=s['stamp'],from_hold_start_s=s['stamp']-start,
        range_m=dd(s),z_m=s['world_xyz'][2],yaw_deg=math.degrees(s['world_yaw']),speed_mps=s['world_speed_mps'])
    def radial_speed_at(stamp):
        s0,s1=near(truth,stamp-.2),near(truth,stamp+.2)
        return (dd(s1)-dd(s0))/(s1['stamp']-s0['stamp']) if s1['stamp']>s0['stamp'] else None
    g=next(s for s in checks['historical_tf_gazebo_and_target_surface']['samples'] if s['request_id']==reply['request_id'])
    return dict(label=case.name,commit=(case/'commit.txt').read_text().strip(),capture_id=reply['request_id'],
        last_capture_sim_s=cap,completed_sim_s=reply['completed_stamp'],done_sim_s=t,
        from_capture_to_done_s=t-cap,reported_distance_at_terminal_m=checks['attempt_observations']['distance_at_terminal_m'],
        done_odometry=dict(measured=odo,closest_dt_s=abs(odo['stamp']-t),mission_position=pmap.tolist()),
        done_setpoint=dict(setpoint=cmd,closest_dt_s=abs(cmd['stamp']-t)),
        done_truth=dict(**world(true_done),closest_dt_s=abs(true_done['stamp']-t),radial_velocity_mps=radial_speed_at(t)),
        capture_truth=dict(**world(phys),closest_dt_s=abs(phys['stamp']-cap),odom=odcap),
        final_cloud_point_map=tmap.tolist(),original_surface_error_m=g['target_surface_error_m'],
        estimated_world_point=etarget.tolist(),true_center_world=center.tolist(),
        estimated_target_horizontal_offset_m=rng(etarget,center),estimated_target_radial_offset_m=float((etarget-center)[:2]@radial),
        independent_surface_error_m=abs(rng(etarget,center)-.05),
        final_production_distance_m=prod,projected_distance_m=rotated,
        raw_frame_tilt_range_m=rotated-prod,cloud_point_versus_rod_axis_range_m=estimate_to_center-rotated,
        body_localization_range_m=actual-estimate_to_center,
        estimate_to_true_axis_m=estimate_to_center,true_minus_production_m=actual-prod,
        pre_land_window=dict(start_sim_s=start,end_sim_s=end,n=len(window),first=world(window[0]),last=world(window[-1]),
            minimum=world(lo),maximum=world(hi),max_outward_displacement_from_first_m=float(hi_delta[:2]@radial),
            tangential_at_max_m=float(hi_delta[:2]@vt),vertical_at_max_m=float(hi_delta[2]),
            true_max_three_dimensional_drift_m=max(math.dist(s['world_xyz'],window[0]['world_xyz']) for s in window),
            true_z_range_m=[min(s['world_xyz'][2] for s in window),max(s['world_xyz'][2] for s in window)],
            yaw_max_change_deg=max(abs(math.degrees(math.remainder(s['world_yaw']-window[0]['world_yaw'],2*math.pi))) for s in window),
            target_center_max_motion_m=max(math.dist(s['target_xyz'],window[0]['target_xyz']) for s in window),
            sp_count=len(setpoints),setpoint_max_displacement_m=max(math.dist(s['position'],setpoints[0]['position']) for s in setpoints),
            max_two_dim_odom_to_setpoint_m=max(math.dist(r['position'][:2],near(setpoints,r['stamp'])['position'][:2])
                                                 for r in od if start<=r['stamp']<=end),
            outside_samples=len(out),first_sample_outside=world(out[0]) if out else None,
            last_sample_outside=world(out[-1]) if out else None,estimated_outside_spans=band_runs(window,nominal,.20)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('cases',type=Path,nargs='+')
    print(json.dumps([audit(c) for c in p.parse_args().cases],indent=2,allow_nan=False))
