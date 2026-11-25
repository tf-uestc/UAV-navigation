#!/usr/bin/env python3
"""Read archived visual trials. No ROS, cloud API, or flight-control calls."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np


def load(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def p95(values):
    return float(np.percentile(values,95)) if values else None


def limits(values):
    return [min(values),max(values)] if values else None


def compact(frames):
    def values(key):
        return [s[key] for s in frames if s.get(key) is not None]
    return dict(n=len(frames),camera_z_m=limits(values('world_camera_z_m')),
        range_m=limits(values('range_m')),yaw_deg=limits(values('world_yaw_deg')),
        sim_time_s=limits(values('stamp')),truth_dt_s=limits(values('truth_lag_s')),
        camera_z_abs_p95_m=p95([abs(x) for x in values('camera_residual_m')]),
        body_z_abs_p95_m=p95([abs(x) for x in values('body_height_residual_m')]),
        camera_minus_body_abs_p95_m=p95([abs(x) for x in values('camera_minus_body_residual_m')]),
        body_matched_n=len(values('body_height_residual_m')),
        level_origin_sensitivity_abs_p95_m=p95([abs(x) for x in values('alignment_sensitivity_m')]),
        tilt_sensitivity_abs_p95_m=p95([abs(x) for x in values('tilt_only_effect_m')]))


def audit_case(case, rootfs, finished, height_tool, heading_tool, frame_tool):
    moving=load(case/'moving.json')
    trace=load(case/'result/trace.json')
    results=load(case/'result/results.json')
    clearance=load(case/'clearance.json')['samples']
    acceptance=load(case/'acceptance.json')
    height=height_tool.audit(case)
    heading=heading_tool.audit(case,rootfs)
    obs=next((s for s in results.get('checks',[]) if s['name']=='attempt_observations'),{})
    mid=obs.get('mission_id')
    def events(kind):
        return [d for _,k,d in trace if k==kind and mid and d.get('mission_id')==mid]
    requests,replies=events('search_capture'),events('detect_result')
    files_ok,frames=frame_tool.inspect_frames(case/'images',requests,replies)
    outcomes=load(case/'images/observations.json')
    stamps=[s['stamp'] for s in moving['samples']]
    maps=[s['map_surface_error_p95_m'] for s in moving['samples']]
    depths=[s['gazebo_surface_error_p95_m'] for s in moving['samples']]
    changes=[s['delayed_tf_max_difference'] for s in moving['samples']]
    derived_map=p95(maps)
    derived_depth=max(depths,default=None)
    derived_tf=max(changes,default=None)
    moving_checks=dict(ten_frames=len(stamps)>=10, map_surface=derived_map is not None and derived_map<=.30,
        depth_surface=derived_depth is not None and derived_depth<=.05,
        historical_tf=derived_tf is not None and derived_tf<1e-8)
    # The final localization runner fits a single ground alignment. The movement observer fits
    # twenty ground samples. Preserve both estimates and report disagreements instead of refitting.
    alignment=np.linalg.inv(moving['ground_alignment_transform'])
    source={s['request_id']:s for check in results.get('checks',[]) if
            check['name']=='historical_tf_gazebo_and_target_surface' for s in check.get('samples',[])}
    capture_rows=[]
    for req,reply in zip(requests,replies):
        stamp=reply['capture_stamp']
        key=(mid,reply['request_id'])
        final=source.get(key[1])
        err=None
        nearest=min(clearance,key=lambda x:abs(x['stamp']-stamp)) if clearance else None
        if reply.get('status')=='SUCCEEDED' and reply.get('point') and nearest and abs(nearest['stamp']-stamp)<=.20:
            world=alignment@list(reply['point']+[1.])
            err=abs(math.dist(world[:2],nearest['target_xyz'][:2])-.05)
        capture_rows.append(dict(request_id=key[1],capture_s=stamp,floor_s=req['floor'],
            completion_s=reply['completed_stamp'],elapsed_s=reply['completed_stamp']-stamp,
            rgb_verified=bool(frames.get(key,{}).get('rgb_ok')),depth_verified=bool(frames.get(key,{}).get('depth_ok')),
            stopped=req.get('stop_confirmed'),status=reply['status'],reason=reply.get('reason'),
            body_heading_deg=math.degrees(nearest['world_yaw']) if nearest and abs(nearest['stamp']-stamp)<=.20 else None,
            body_to_target_axis_m=math.dist(nearest['world_xyz'][:2],nearest['target_xyz'][:2])
                if nearest and abs(nearest['stamp']-stamp)<=.20 else None,
            truth_dt_s=abs(nearest['stamp']-stamp) if nearest else None,
            original_target_error_m=final.get('target_surface_error_m') if final else None,
            original_truth_dt_s=final.get('truth_time_delta_s') if final else None,
            twenty_ground_samples_target_error_m=err))
    states=events('mission')
    done=next((i for i,s in enumerate(states) if s['phase']=='DONE'),None)
    latest=capture_rows[-1] if capture_rows else {}
    before=states[done-1] if done is not None and done>0 else {}
    last_fresh=bool(latest and done is not None and before.get('phase')=='WAIT_VLM' and
        before.get('detection_request_id')==latest['request_id'] and latest['status']=='SUCCEEDED' and
        latest['completion_s']<=states[done]['observed_sim']+.001 and requests[-1]['request_id']==latest['request_id'])
    accepted=bool(last_fresh and latest.get('original_target_error_m') is not None and latest['original_target_error_m']<=.20)
    independent=bool(last_fresh and latest.get('twenty_ground_samples_target_error_m') is not None and latest['twenty_ground_samples_target_error_m']<=.20)
    def matches_optional(derived, stored):
        if derived is None or stored is None:
            return derived is None and stored is None
        return abs(derived-stored)<=1e-10
    camera_agreement=(matches_optional(derived_map,moving.get('map_surface_error_p95_of_frame_p95_m')) and
        matches_optional(derived_depth,moving.get('depth_surface_max_frame_p95_m')) and
        all(moving_checks.values())==moving.get('passed'))
    summary=dict(label=case.name,commit=height['commit'],distance=height['nominal_m'],direction=height['direction'],
        official_passed=finished.get('passed') is True,official_errors=acceptance.get('errors',[]),
        terminal=obs.get('terminal'),terminal_reason=obs.get('reason'),flight_diagnostic=results.get('error'),
        moving_passed=all(moving_checks.values()),moving_checks=moving_checks,
        moving_original_report_agrees=camera_agreement,frame_count=len(stamps),height=compact(height['frames']),
        ground_alignment_samples=moving.get('ground_alignment_samples'),fixed_target_radius_m=.05,
        map_surface_p95_m=derived_map,depth_surface_max_frame_p95_m=derived_depth,
        delayed_tf_max_difference=derived_tf,capture_counts=[len(requests),len(replies),len(outcomes['rows'])],
        all_captures_match=files_ok,captures=capture_rows,final_reply_precedes_done=last_fresh,
        final_original_target_pass=accepted,final_independent_target_pass=independent,
        heading_gate_passed=heading['pass_heading_gate'],heading_at_each_request=heading['gate_samples'],
        heading_error_deg=heading['summary'].get('cam_body_heading_deg'),late_resets=heading['late_heading_resets'],
        px4_mag_type=heading['ekf2_mag_type'],ulog_sha256=heading['ulg_sha256'],
        hold_body_z_median_m=(height['hold_body_height_residual_m'] or {}).get('median'))
    return summary,height['frames']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path)
    parser.add_argument('--rootfs',type=Path,required=True)
    parser.add_argument('--batch',required=True)
    parser.add_argument('--helper-dir',action='append',type=Path,default=[])
    args=parser.parse_args()
    for directory in reversed(args.helper_dir):
        sys.path.insert(0,str(directory))
    import diagnose_yaw, inspect_camera_height, summarize_visual_attempts
    source=args.archive/(args.batch+'-attempts.jsonl')
    ledger=[json.loads(l) for l in source.read_text().splitlines() if l]
    starts=[x['label'] for x in ledger if x['event']=='start']
    finished={x['label']:x for x in ledger if x['event']=='finish'}
    expected={f'{args.batch}-{distance}-{direction}-{repeat}' for distance,directions in
              [('2.4',('front','left45','right45','left90','right90')),
               ('1.5',('front','left45','right45'))] for direction in directions for repeat in (1,2)}
    attempts=[];all_frames=[];by_case=defaultdict(list)
    for label in starts:
        if label not in finished:
            attempts.append(dict(label=label,error='uncompleted ledger entry'))
            continue
        try:
            attempt,frames=audit_case(args.archive/'flights'/label,args.rootfs,finished[label],
                                      inspect_camera_height,diagnose_yaw,summarize_visual_attempts)
            attempts.append(attempt)
            all_frames.extend(frames)
            by_case[f"{attempt['distance']}/{attempt['direction']}"].extend(frames)
        except (OSError,KeyError,ValueError,TypeError,IndexError,StopIteration,np.linalg.LinAlgError) as exc:
            attempts.append(dict(label=label,error=str(exc),official_passed=finished[label].get('passed')))
    report=dict(batch=args.batch,final=len(starts)==len(expected) and set(starts)==set(finished)==expected,
        missing=sorted(expected-set(finished)),unexpected=sorted(set(starts)-expected),
        attempts=attempts,overall=compact(all_frames),
        by_case={k:compact(v) for k,v in by_case.items()},
        by_measured_range={label:compact([s for s in all_frames if low<=s['range_m']<high])
            for label,low,high in [('under_2m',0,2),('2_to_under_3m',2,3),('at_least_3m',3,math.inf)]},
        ledger_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    print(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2))
    if not report['final'] or any('error' in x for x in attempts):
        raise SystemExit(2)


if __name__=='__main__':
    main()
