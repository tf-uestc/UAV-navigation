#!/usr/bin/env python3
"""Read a visual acceptance archive and independently check each numbered flight."""
import argparse
from collections import Counter, defaultdict
from fnmatch import fnmatchcase
import json
import math
from pathlib import Path
import re
from zipfile import BadZipFile

import numpy as np


def png_shape(path):
    try:
        from PIL import Image
    except ImportError:
        import cv2
        image=cv2.imread(str(path),cv2.IMREAD_COLOR)
        return image.shape[:2] if image is not None else None
    with Image.open(path) as image:
        image.load()
        return (image.height,image.width) if image.format=='PNG' else None


LABEL = re.compile(r"(.+)-(2\.4|1\.5)-(front|left45|right45|left90|right90)-(\d+)$")
ANGLES = dict(front=0, left45=45, right45=-45, left90=90, right90=-90)


def read(path, default):
    return json.loads(path.read_text(encoding='utf-8-sig')) if path.is_file() else default


def limits(values):
    return [min(values), max(values)] if values else None


def yaw_difference_deg(a, b):
    return abs(math.degrees(math.remainder(a-b, 2*math.pi)))


def sample_window(values, start, end, field, max_gap):
    """Cover both ends; a lone good status cannot certify a ten-second interval."""
    times = [d[field] for d in values]
    return (len(times) >= 2 and all(0 <= b-a <= max_gap for a,b in zip(times,times[1:]))
            and times[0]-start <= max_gap and end-times[-1] <= max_gap)


def inspect_frames(folder, requests, replies):
    captured = read(folder/'observations.json', {})
    rows = captured.get('rows', [])
    frames = {}
    for row in rows:
        req, res = row['request'], row['result']
        key = (req['mission_id'], req['request_id'])
        if key in frames or key != (res['mission_id'],res['request_id']):
            return False, frames
        name = row.get('exact_frame')
        depth = row.get('exact_depth')
        matched = bool(name and Path(name).name==name and any(
            item['file']==name and abs(item['stamp']-res['capture_stamp'])<.0005
            for item in row['frames']))
        shape = None
        if matched and (folder/name).is_file():
            shape=png_shape(folder/name)
        depth_ok = False
        if depth and Path(depth).name==depth and (folder/depth).is_file():
            expected = '%s-%s-%d-depth.npz' % (*key, round(res['capture_stamp']*1e9))
            if depth == expected:
                with np.load(folder/depth,allow_pickle=False) as samples:
                    depth_ok = samples['depth'].shape==shape
        frames[key] = dict(rgb_ok=bool(shape),depth_ok=depth_ok,stamp=res['capture_stamp'],
                           status=res['status'],reason=res['reason'])
    req_ids = [(d['mission_id'],d['request_id']) for d in requests]
    reply_ids = [(d['mission_id'],d['request_id']) for d in replies]
    ok = bool(req_ids) and len(set(req_ids))==len(req_ids) and req_ids==reply_ids
    ok = ok and set(req_ids)==set(frames) and not captured.get('pending')
    for d,req in zip(replies,requests):
        frame = frames.get((d['mission_id'],d['request_id']),{})
        ok = (ok and frame.get('rgb_ok') and frame.get('depth_ok') and frame['stamp']==d['capture_stamp']
              and frame['status']==d['status'] and frame['reason']==d['reason']
              and req.get('stop_confirmed') and d['capture_stamp'] >= req['floor']-.001
              and 0 <= d['completed_stamp'] - d['capture_stamp'] <= 42.1)
    return bool(ok), frames


def contact_pairs(path):
    """Stream the Gazebo protobuf text log; it can be hundreds of MB per flight."""
    first=None
    witness=0
    hazards=set()
    permitted={'ground_plane::link::collision','landing_pad::link::pad_collision'}
    with path.open('rb') as stream:
        for line in stream:
            if b'collision1:' in line:
                first=line.split(b'"')[1].decode()
            if b'collision2:' in line and first:
                second=line.split(b'collision2:',1)[1].split(b'"')[1].decode()
                robot=[v.startswith('iris_depth_camera::') for v in (first,second)]
                if any(robot) and not all(robot):
                    other=second if robot[0] else first
                    if other not in permitted:
                        hazards.add(other)
                if 'contact_witness::' in first+second and 'ground_plane::' in first+second:
                    witness+=1
                first=None
    return witness,sorted(hazards)


def inspect(case, match, finished):
    batch, nominal, direction, repeat = match.groups()
    nominal=float(nominal)
    row=dict(label=case.name,batch=batch,nominal_m=nominal,direction=direction,repeat=int(repeat),
             path=str(case),completed=finished is not None,exit_code=(finished or {}).get('exit_code'),
             individually_verified=False)
    checks={}
    try:
        result=read(case/'result/results.json',{})
        audit=read(case/'acceptance.json',{})
        commit=case/'commit.txt'
        row['commit']=commit.read_text().strip() if commit.is_file() else None
        row['runner_error']=result.get('error')
        row['auditor_passed']=audit.get('passed')
        row['auditor_errors']=audit.get('errors',[])
        obs=next((s for s in result.get('checks',[]) if s['name']=='attempt_observations'),{})
        mid=obs.get('mission_id')
        trace=read(case/'result/trace.json',[])
        def events(kind):
            return [d for _,k,d in trace if k==kind]
        def mission_events(kind):
            return [d for d in events(kind) if mid and d.get('mission_id')==mid]
        requests=mission_events('search_capture')
        replies=mission_events('detect_result')
        launch=case/'launch.log'
        if launch.is_file():
            with launch.open(encoding='utf-8',errors='replace') as handle:
                row['cloud_calls']=sum('Querying DashScope (' in line for line in handle)
        else:
            row['cloud_calls']=None
        row.update(requests=len(requests),responses=len(replies),response_counts=dict(Counter(
            r.get('reason') or r.get('status') for r in replies)),terminal=obs.get('terminal'),
            terminal_reason=obs.get('reason'),segments=obs.get('segments'),scans=obs.get('scans'),
            distance_at_terminal_m=obs.get('distance_at_terminal_m'))
        checks['all_requests_accounted']=(row['cloud_calls'] is not None
            and row['cloud_calls']==len(requests)==len(replies) and 0<len(requests)<=12)
        phases=mission_events('mission')
        done_index=next((i for i,state in enumerate(phases) if state['phase']=='DONE'),None)
        done=phases[done_index] if done_index is not None else {}
        previous=phases[done_index-1] if done_index else {}
        done_stamp=done.get('observed_sim')
        hold=next((s for s in result.get('checks',[]) if s['name']=='observed_and_held'),{})
        start,end=hold.get('start_sim'),hold.get('end_sim')
        clearance=read(case/'clearance.json',{})
        samples=clearance.get('samples',[])
        radius=clearance.get('vehicle_bounding_radius_m')
        if samples and radius is not None:
            row['minimum_rod_clearance_m']=min(
                math.dist(s['world_xyz'][:2],s['target_xyz'][:2])-.05-radius for s in samples)
        checks['rod_clearance']=row.get('minimum_rod_clearance_m',0)>.5
        checks['reached']=bool(done and done.get('reason')=='observation_standoff_reached'
                               and obs.get('segments',0)>0 and obs.get('terminal')=='DONE')
        scans=[d['scan_offset_deg'] for d in mission_events('mission') if d['phase']=='SCAN_TURN']
        checks['search_order']=scans==[45,-45,90,-90][:len(scans)] and (
            direction=='front' or ANGLES[direction] in scans)
        if done_stamp is not None and samples:
            at=min(samples,key=lambda x: abs(x['stamp']-done_stamp))
            true_distance=math.dist(at['world_xyz'][:2],at['target_xyz'][:2])
            row['independent_terminal_distance_m']=true_distance
            row['terminal_truth_dt_s']=abs(at['stamp']-done_stamp)
            checks['terminal_distance']=row['terminal_truth_dt_s']<=.30 and abs(true_distance-nominal)<=.20
        else:
            checks['terminal_distance']=False
        land=[d['observed_sim'] for d in events('flight_command') if d.get('command')=='LAND' and 'observed_sim' in d]
        checks['pre_land_hold']=bool(done_stamp is not None and start is not None and end is not None
                                     and done_stamp<=start and end-start>=10 and land and min(land)>=end)
        fcu=events('fcu')
        row['last_fcu']=fcu[-1] if fcu else None
        exit_file=case/'flight.exit'
        row['postflight_ground_check']=exit_file.read_text().strip() == '0' if exit_file.is_file() else None
        state_path=case/'postflight-state.json'
        if state_path.is_file():
            state=read(state_path,{}).get('state') or {}
            row['postflight_ground_check']=(state.get('connected') is True and
                                           state.get('landed') is True and state.get('armed') is False)
        checks['landing']=bool(any(s.get('armed') for s in fcu)
                               and row['postflight_ground_check'] and fcu[-1].get('connected')
                               and fcu[-1].get('armed') is False
                               and result.get('final_armed') is False and 'landed_disarmed' in
                               [s['name'] for s in result.get('checks',[])])
        held=[s for s in samples if start is not None and end is not None and start<=s['stamp']<=end]
        row.update(hold_count=len(held),hold_seconds=None,truth_max_speed_mps=None,
                   truth_max_yaw_change_deg=None,truth_max_drift_m=None)
        if held:
            dist=[math.dist(s['world_xyz'][:2],s['target_xyz'][:2]) for s in held]
            row.update(hold_seconds=held[-1]['stamp']-held[0]['stamp'],hold_distance_range_m=limits(dist),
                       truth_max_drift_m=max(math.dist(s['world_xyz'],held[0]['world_xyz']) for s in held))
            if all('world_speed_mps' in s and 'world_yaw' in s for s in held):
                row.update(truth_max_speed_mps=max(s['world_speed_mps'] for s in held),
                           truth_max_yaw_change_deg=max(yaw_difference_deg(s['world_yaw'],held[0]['world_yaw']) for s in held),
                           truth_start_yaw_deg=math.degrees(held[0]['world_yaw']),
                           truth_max_target_bearing_error_deg=max(yaw_difference_deg(s['world_yaw'],
                               math.atan2(s['target_xyz'][1]-s['world_xyz'][1],s['target_xyz'][0]-s['world_xyz'][0])) for s in held))
            checks['hold_truth']=bool(len(held)>=70 and row['hold_seconds']>=10
                and sample_window(held,start,end,'stamp',.30) and row['truth_max_drift_m']<=.30
                and max(abs(v-nominal) for v in dist)<=.20 and row['truth_max_speed_mps'] is not None
                and row['truth_max_speed_mps']<=.15 and row['truth_max_yaw_change_deg']<=10)
        else:
            checks['hold_truth']=False
        status=[s for s in events('flight') if held and start<=s.get('observed_sim',-1)<=end]
        odom=[s for s in events('odom') if held and start<=s['stamp']<=end]
        checks['hold_control']=bool(held and sample_window(status,start,end,'observed_sim',.7)
            and sample_window(odom,start,end,'stamp',.5) and all(s['mode']=='HOLD' and
            s.get('context') is None and s.get('owns_flight') and s.get('yaw_reference') is not None for s in status))
        if checks['hold_control']:
            row['max_heading_reference_error_deg']=max(yaw_difference_deg(o['yaw'],min(status,key=lambda s:
                abs(s['observed_sim']-o['stamp']))['yaw_reference']) for o in odom)
            checks['hold_control']=row['max_heading_reference_error_deg']<=10
        try:
            images,frames=inspect_frames(case/'images',requests,replies)
        except (OSError,ValueError,KeyError,TypeError,EOFError,BadZipFile) as exc:
            row['image_error']=str(exc)
            images,frames=False,{}
        checks['exact_observations']=images
        row['matched_rgb_count']=sum(v['rgb_ok'] for v in frames.values())
        row['matched_depth_count']=sum(v['depth_ok'] for v in frames.values())
        local=next((s['samples'] for s in result.get('checks',[]) if s['name']=='historical_tf_gazebo_and_target_surface'),[])
        final=replies[-1] if replies else {}
        last=next((s for s in local if s.get('request_id') is not None and
                   s['request_id']==final.get('request_id')),None)
        row['last_request_id']=requests[-1]['request_id'] if requests else None
        row['last_verified_target_error_m']=last.get('target_surface_error_m') if last else None
        checks['latest_reply']=bool(final.get('status')=='SUCCEEDED' and last
            and final['request_id']==row['last_request_id'] and last['capture_stamp']==final['capture_stamp']
            and previous.get('phase')=='WAIT_VLM' and previous.get('detection_request_id')==final['request_id']
            and done_stamp is not None and final['completed_stamp']<=done_stamp+.001
            and last['truth_time_delta_s']<=.075 and last['target_surface_error_m']<=.20)
        moving=read(case/'moving.json',{})
        frames=moving.get('samples',[])
        row['moving_rgbd_count']=len(frames)
        row['camera_world_z_range_m']=limits([f['truth_camera_z_m'] for f in frames if 'truth_camera_z_m' in f])
        row['camera_target_range_m']=limits([f['camera_to_target_horizontal_m'] for f in frames if 'camera_to_target_horizontal_m' in f])
        height_errors=[abs(f['estimated_world_camera_z_m']-f['truth_camera_z_m'])
                       for f in frames if 'estimated_world_camera_z_m' in f and 'truth_camera_z_m' in f]
        row['camera_abs_z_error_p95_m']=float(np.percentile(height_errors,95)) if height_errors else None
        row['camera_abs_z_error_max_m']=max(height_errors,default=None)
        row['moving_rgbd_passed']=moving.get('passed')
        checks['moving_camera']=bool(moving.get('passed') is True and len(frames)>=10
            and len(height_errors)==len(frames) and row['camera_target_range_m'] is not None
            and all({'gazebo_surface_error_p95_m','map_surface_error_p95_m','delayed_tf_max_difference',
                     'camera_to_target_horizontal_m'} <= frame.keys() for frame in frames)
            and max(frame['delayed_tf_max_difference'] for frame in frames)<1e-8
            and max(frame['gazebo_surface_error_p95_m'] for frame in frames)<=.05
            and np.percentile([frame['map_surface_error_p95_m'] for frame in frames],95)<=.30)
        physics=read(case/'physics-contacts.json',{})
        pair_log=case/'physics-contacts.txt'
        witness,hazards=contact_pairs(pair_log) if pair_log.is_file() else (None,None)
        row['unexpected_contacts']=hazards
        checks['contact_evidence']=bool(witness and not hazards and physics.get('passed') is True
                                        and physics.get('physics_messages',0)>200)
        odom_all=events('odom')
        checks['contact_coverage']=bool(odom_all and
            physics.get('start_stamp',math.inf)<=odom_all[0]['stamp']+.5 and
            physics.get('last_message_stamp',-math.inf)>=odom_all[-1]['stamp']-.3)
        checks['runner']=finished is not None and finished.get('exit_code')==0 and result.get('passed') is True
        checks['version']=bool(row['commit'])
    except (OSError,TypeError,ValueError,KeyError,IndexError,EOFError,BadZipFile) as exc:
        row['evidence_error']=str(exc)
        checks['read_all_evidence']=False
    row['checks']=checks
    row['failed_checks']=[k for k,v in checks.items() if not v]
    row['individually_verified']=bool(checks and all(checks.values()))
    row['accepted_by_both']=row['individually_verified'] and row.get('auditor_passed') is True
    row['audit_disagreement'] = (row.get('auditor_passed') is not None and
                                 row.get('auditor_passed') != row['individually_verified'])
    return row


def summarize(rows):
    return dict(started=len(rows),completed=sum(r['completed'] for r in rows),
                passed=sum(r['accepted_by_both'] for r in rows),
                independently_verified=sum(r['individually_verified'] for r in rows),
                auditor_passed=sum(r.get('auditor_passed') is True for r in rows),
                cloud_calls_logged=sum(r.get('cloud_calls') or 0 for r in rows),
                attempts_missing_cloud_log=sum(r.get('cloud_calls') is None for r in rows),
                success_rate=(sum(r['accepted_by_both'] for r in rows)/len(rows)) if rows else None)


def report(roots, patterns):
    rows=[]
    ledger_warnings=[]
    for root in roots:
        if not root.is_dir():
            raise FileNotFoundError(f'Archive directory does not exist: {root}')
        status={}
        for ledger in root.glob('*-attempts.jsonl'):
            lines=ledger.read_text(encoding='utf-8-sig').splitlines(keepends=True)
            for number,line in enumerate(lines,1):
                try:
                    event=json.loads(line)
                except json.JSONDecodeError:
                    if number == len(lines) and not line.endswith('\n'):
                        ledger_warnings.append(f'{ledger}:{number} unfinished last event')
                        break
                    raise ValueError(f'Invalid ledger at {ledger}:{number}') from None
                status[event['label']]=event if event['event']=='finish' else None
        folder=root/'flights'
        labels=set(status) | ({p.name for p in folder.iterdir() if p.is_dir()} if folder.exists() else set())
        for label in sorted(labels):
            m=LABEL.fullmatch(label)
            if m and any(fnmatchcase(m[1],p) for p in patterns):
                rows.append(inspect(folder/label,m,status.get(label)))
    groups={}
    for group,keys in dict(by_batch=('batch',),by_distance=('nominal_m',),
                           by_version_total=('commit',),by_version_distance=('commit','nominal_m'),
                           by_direction=('nominal_m','direction'),by_version=('commit','nominal_m','direction')).items():
        ordered=defaultdict(list)
        for row in rows:
            ordered['/'.join(str(row.get(k) or 'unknown') for k in keys)].append(row)
        groups[group]={k:summarize(v) for k,v in ordered.items()}
    return dict(overall=summarize(rows),attempts=rows,ledger_warnings=ledger_warnings,**groups)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots',type=Path,nargs='+',help='Archive root(s) with flights/ and batch ledgers')
    parser.add_argument('--batch',action='append',help="Batch glob; repeat as needed, default all")
    parser.add_argument('--out',type=Path,help='Write summary JSON here; default is stdout')
    args=parser.parse_args()
    data=report(dict.fromkeys(p.resolve() for p in args.roots),args.batch or ['*'])
    text=json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if args.out:
        args.out.write_text(text,encoding='utf-8')
        print(json.dumps(data['overall']))
    else:
        print(text,end='')


if __name__=='__main__':
    main()
