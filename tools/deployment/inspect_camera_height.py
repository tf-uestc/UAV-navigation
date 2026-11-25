#!/usr/bin/env python3
"""Reconstruct body and camera height residuals from saved Gazebo flights only."""
import argparse
import json
import math
from pathlib import Path
import re

import numpy as np


def read(path):
    return json.loads(path.read_text())


def statistics(values):
    if not len(values):
        return None
    values = np.asarray(values, dtype=float)
    return dict(n=len(values), low=float(min(values)), median=float(np.median(values)),
                high=float(max(values)), abs_p95=float(np.percentile(np.abs(values), 95)))


def interpolate(samples, t, max_gap):
    """Interpolate only between timestamps covered by this recorder."""
    stamps, positions = samples
    end = int(np.searchsorted(stamps, t))
    if end < len(stamps) and stamps[end] == t:
        return positions[end]
    if end == 0 or end == len(stamps) or stamps[end]-stamps[end-1] > max_gap:
        return None
    fraction = (t-stamps[end-1]) / (stamps[end]-stamps[end-1])
    return positions[end-1] + fraction*(positions[end]-positions[end-1])


def table(samples, field):
    samples = sorted(samples, key=lambda s: s['stamp'])
    return ([s['stamp'] for s in samples], np.array([s[field] for s in samples]))


def audit(case):
    moving = read(case/'moving.json')
    trace = read(case/'result/trace.json')
    truth = read(case/'clearance.json')['samples']
    odoms = [d for _,kind,d in trace if kind == 'odom']
    body_map = table(odoms, 'position')
    body_world = table(truth, 'world_xyz')
    fitted = np.array(moving['ground_alignment_transform'])
    matrix = np.linalg.inv(fitted)
    arms = [d['observed_sim'] for _,kind,d in trace if kind == 'fcu' and d['armed']]
    preflight = []
    if arms:
        for s in truth:
            if s['stamp'] >= min(arms) or s['world_speed_mps'] > .05:
                continue
            position = interpolate(body_map, s['stamp'], .20)
            if position is not None:
                preflight.append((position, s['world_xyz']))
    # Use the same ground position for a separate purely vertical datum.
    # Any resulting discrepancy measures sensitivity to the fitted roll/pitch.
    level_offset = np.median([p[2]-w[2] for p,w in preflight]) if preflight else None
    ground_xyz = np.median([w for _,w in preflight],axis=0) if preflight else None
    matched_offset = float((fitted @ [*ground_xyz,1])[2]-ground_xyz[2]) if preflight else None

    def residuals(stamp, truth_lag=0):
        pos = interpolate(body_map, stamp, .20)
        world = interpolate(body_world, stamp + truth_lag, .35)
        if pos is None or world is None:
            return None
        estimated = (matrix @ [*pos,1])[:3]
        return dict(body_height_residual_m=float(estimated[2]-world[2]),
                    body_world_z_m=float(world[2]),
                    level_body_residual_m=float(pos[2]-level_offset-world[2]) if preflight else None,
                    level_same_ground_origin_residual_m=float(pos[2]-matched_offset-world[2]) if preflight else None)

    frames = []
    for s in moving['samples']:
        t, lag = s['stamp'], s['truth_time_offset_s']
        same = residuals(t)
        later = residuals(t,lag)
        error = s['estimated_world_camera_z_m'] - s['truth_camera_z_m']
        frame = dict(stamp=t, camera_residual_m=error, truth_lag_s=lag,
                     speed_mps=s['speed_mps'], world_yaw_deg=s['world_camera_yaw_deg'],
                     world_camera_z_m=s['truth_camera_z_m'], range_m=s['camera_to_target_horizontal_m'])
        if same and later:
            frame.update(later)
            frame['capture_body_residual_m'] = same['body_height_residual_m']
            frame['camera_minus_body_residual_m'] = error - later['body_height_residual_m']
            frame['body_truth_lag_effect_m'] = same['body_height_residual_m']-later['body_height_residual_m']
            if preflight:
                frame['alignment_sensitivity_m'] = later['body_height_residual_m']-later['level_body_residual_m']
                frame['tilt_only_effect_m'] = later['body_height_residual_m']-later['level_same_ground_origin_residual_m']
        frames.append(frame)

    ground = [float((matrix @ [*p,1])[2]-w[2]) for p,w in preflight]
    run = read(case/'result/results.json')
    hold = next((c for c in run.get('checks',[]) if c['name']=='observed_and_held'),{})
    hold_truth = []
    for s in truth:
        if hold.get('start_sim',math.inf)<=s['stamp']<=hold.get('end_sim',-math.inf):
            residual = residuals(s['stamp'])
            if residual:
                hold_truth.append((residual['body_height_residual_m'],s['world_speed_mps']))
    requests = {(d['mission_id'],d['request_id']): d for _,kind,d in trace if kind == 'search_capture'}
    captures = []
    images = case/'images/observations.json'
    if images.is_file():
        for row in read(images)['rows']:
            d = row['result']
            t = d['capture_stamp']
            req = requests.get((d['mission_id'],d['request_id']),{})
            rgb = row.get('exact_frame')
            depth = row.get('exact_depth')
            nearest = min(truth,key=lambda s: abs(s['stamp']-t)) if truth else None
            capture = dict(request_id=d['request_id'],stamp=t,status=d['status'],
                           scan_deg=req.get('scan_offset_deg'),exact_rgb=bool(rgb and (images.parent/rgb).is_file()),
                           exact_depth=bool(depth and (images.parent/depth).is_file()))
            if nearest and abs(nearest['stamp']-t) <= .20:
                capture.update(world_yaw_deg=math.degrees(nearest['world_yaw']),
                               body_to_target_m=nearest['horizontal_distance_m'])
            capture.update(residuals(t) or {})
            captures.append(capture)
    version=case/'commit.txt'
    match=re.fullmatch(r'(.+)-(\d+\.\d+)-(front|left45|right45|left90|right90)-(\d+)',case.name)
    return dict(label=case.name,commit=version.read_text().strip() if version.is_file() else None,
                nominal_m=float(match[2]) if match else None,direction=match[3] if match else None,
                frame_count=len(frames),ground_samples_fit=moving['ground_alignment_samples'],
                ground_recheck_count=len(ground),ground_recheck_residual_m=statistics(ground),
                ground_map_z_minus_world_z_m=float(level_offset) if preflight else None,
                fitted_tilt_deg=math.degrees(math.acos(float(np.clip(matrix[2,2],-1,1)))),
                hold_body_height_residual_m=statistics([err for err,_ in hold_truth]),
                hold_world_speed_mps=statistics([speed for _,speed in hold_truth]),
                map_horizontal_surface_p95_m=moving.get('map_surface_error_p95_of_frame_p95_m'),
                depth_horizontal_surface_worst_frame_p95_m=moving.get('depth_surface_max_frame_p95_m'),
                frames=frames,captures=captures)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('flights',type=Path)
    parser.add_argument('--pattern',default='V03*')
    args = parser.parse_args()
    attempts = []
    pending = []
    for case in sorted(args.flights.glob(args.pattern)):
        if not case.is_dir():
            continue
        try:
            attempts.append(audit(case))
        except (OSError,KeyError,TypeError,ValueError,np.linalg.LinAlgError) as exc:
            pending.append(dict(label=case.name,reason=str(exc)))
    def summarize(frames):
        fields=('camera_residual_m','body_height_residual_m','camera_minus_body_residual_m',
                'body_truth_lag_effect_m','alignment_sensitivity_m','tilt_only_effect_m','range_m','world_camera_z_m',
                'world_yaw_deg','truth_lag_s','speed_mps')
        return {key:statistics([s[key] for s in frames if key in s]) for key in fields}
    for case in attempts:
        case['summary'] = summarize(case['frames'])
    frames = [s for case in attempts for s in case['frames']]
    grouped = {}
    for label,low,high in [('under 2m',0.,2.),('2-3m',2.,3.),('3m and above',3.,math.inf)]:
        grouped[label] = summarize([s for s in frames if low <= s['range_m'] < high])
    cells={}
    for case in attempts:
        key=f"{case['nominal_m']}/{case['direction']}"
        cells.setdefault(key,[]).extend(case['frames'])
    print(json.dumps(dict(flights=str(args.flights),overall=summarize(frames),by_measured_range=grouped,
                          by_scheduled_case={k:summarize(v) for k,v in cells.items()},attempts=attempts,
                          pending=pending),indent=2,allow_nan=False))


if __name__ == '__main__':
    main()
