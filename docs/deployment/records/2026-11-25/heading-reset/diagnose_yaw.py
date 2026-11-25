#!/usr/bin/env python3
"""Read only. Compare a flight's recorded PX4 ULog, MAVROS and Gazebo truth."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
from pyulog import ULog


def read(p):
    return json.loads(p.read_text())


def deg(a):
    return float(np.rad2deg(a))


def wrap(a):
    return (a+180.)%360.-180.


def stats(a):
    if not len(a):
        return None
    return dict(n=len(a),median=float(np.median(a)),minimum=float(np.min(a)),maximum=float(np.max(a)),
                abs_p95=float(np.percentile(np.abs(a),95)))


def yaw_signal(rows, field):
    return np.array([r['stamp'] for r in rows]),np.unwrap([r[field] for r in rows])


def sample(signal,t,limit=.35):
    times,vals=signal
    j=int(np.searchsorted(times,t))
    if j<len(times) and times[j]==t:
        return vals[j]
    if not (0<j<len(times)) or times[j]-times[j-1]>limit:
        return None
    return vals[j-1]+(vals[j]-vals[j-1])*(t-times[j-1])/(times[j]-times[j-1])


def transitions(dataset,key):
    v=dataset[key]
    idx=np.r_[0,np.flatnonzero(v[1:]!=v[:-1])+1]
    return [(float(dataset['timestamp'][i])/1e6,int(v[i])) for i in idx]


def audit(case,rootfs):
    supervisor=Path(read(case/'start.json')['owner']['log'])
    px4log=(supervisor.parent/'px4.log').read_text(errors='replace')
    ref=re.search(r'Opened full log file:\s*(\./log/[^\s]+\.ulg)',px4log)
    if not ref:
        raise ValueError('This supervisor log does not identify the ULog')
    path=rootfs/ref[1]
    topics=['vehicle_local_position','vehicle_attitude','estimator_status_flags','vehicle_status',
            'estimator_event_flags','estimator_status','yaw_estimator_status']
    u=ULog(str(path),message_name_filter_list=topics)
    pos=u.get_dataset('vehicle_local_position').data
    att=u.get_dataset('vehicle_attitude').data
    flags=u.get_dataset('estimator_status_flags').data
    vst=u.get_dataset('vehicle_status').data
    es=u.get_dataset('estimator_status').data
    ekf_events=u.get_dataset('estimator_event_flags').data
    yaw_filter=u.get_dataset('yaw_estimator_status').data
    moving=read(case/'moving.json')
    truth=read(case/'clearance.json')['samples']
    trace=read(case/'result/trace.json')
    od=sorted([s for _,kind,s in trace if kind=='odom'],key=lambda s:s['stamp'])
    mission=[s for _,kind,s in trace if kind=='mission']
    captures=[s for _,kind,s in trace if kind=='search_capture']
    good=transitions(pos,'heading_good_for_control')
    ready=next((t for t,state in good if state),None)
    waits=[s['observed_sim'] for s in mission if s['phase']=='WAIT_VLM']
    cloud=waits[0] if waits else None
    takeoff=next(t for t,s in transitions(flags,'cs_in_air') if s and t>3)
    armed=next(t for t,s in transitions(vst,'arming_state') if s==2)
    precheck=next((t for t,s in transitions(vst,'pre_flight_checks_pass') if s),None)
    desired=np.linalg.inv(moving['ground_alignment_transform'])
    align=deg(math.atan2(desired[1,0],desired[0,0]))
    os=yaw_signal(od,'yaw')
    gs=yaw_signal(truth,'world_yaw')
    att_time=att['timestamp'].astype(float)/1e6
    qw,qx,qy,qz=[att['q[%d]'%i] for i in range(4)]
    ned=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    attitude=(att_time,np.unwrap(math.pi/2-ned))
    resets=[]
    for i in np.flatnonzero(np.diff(pos['heading_reset_counter'])!=0)+1:
        t=float(pos['timestamp'][i])/1e6
        k=int(np.abs(att_time-t).argmin())
        # A quaternion reset may be published one IMU cycle after the local position reset.
        candidates=np.flatnonzero((np.abs(att_time-t)<=.04)&(att['quat_reset_counter']==pos['heading_reset_counter'][i]))
        q=min(candidates,key=lambda j:abs(att_time[j]-t)) if len(candidates) else k
        phase=next((e for e in reversed(mission) if e.get('observed_sim',1e6)<=t),{})
        prior=int(np.searchsorted(yaw_filter['timestamp']/1e6,t)-1)
        g_before=sample(gs,t-.15)
        g_after=sample(gs,t+.15)
        resets.append(dict(stamp=t, counter=int(pos['heading_reset_counter'][i]),
            ned_delta_heading_deg=deg(pos['delta_heading'][i]),quat_stamp=float(att_time[q]),
            quat_counter=int(att['quat_reset_counter'][q]),q_after=[float(att[f'q[{j}]'][q]) for j in range(4)],
            delta_q_reset=[float(att[f'delta_q_reset[{j}]'][q]) for j in range(4)],
            estimated_hagl_m=float(pos['dist_bottom'][i]),phase=phase.get('phase'),
            scan_offset=phase.get('scan_offset_deg'),
            world_yaw_rate_at_reset_deg_s=deg(g_after-g_before)/.3 if g_before is not None and g_after is not None else None,
            yaw_gsf_valid_before=bool(yaw_filter['yaw_composite_valid'][prior]) if prior>=0 else None,
            yaw_gsf_variance_before=float(yaw_filter['yaw_variance'][prior]) if prior>=0 else None))
    gyro_yaw=[]
    ground_early=[];ground_late=[];air_before=[];post=[]
    reset_t=next((r['stamp'] for r in resets if r['stamp']>takeoff),None)
    for s in od:
        t=s['stamp'];gv=sample(gs,t)
        if gv is None:
            continue
        e=wrap(deg(s['yaw']-gv)+align)
        if t<armed and abs(s['speed'])<.05:
            (ground_late if precheck is not None and t>=precheck else ground_early).append(e)
        if takeoff<=t and reset_t is not None and t<reset_t-.25:
            air_before.append(e)
        if reset_t is not None and t>=reset_t+.6:
            post.append(e)
        q=sample(attitude,t)
        if q is not None:
            gyro_yaw.append(wrap(deg(s['yaw']-q)))
    gr_late=np.median(ground_late) if ground_late else None
    rows=[]
    for s in moving['samples']:
        t=s['stamp'];a=sample(os,t); g=sample(gs,t);gl=sample(gs,t+s['truth_time_offset_s'])
        if any(x is None for x in (a,g,gl)):
            continue
        bias=wrap(deg(a)+align-s['world_camera_yaw_deg'])
        delta=desired[:3,:3]@s['camera_xyz_error_m']
        h=math.radians(s['world_camera_yaw_deg'])
        r=s['camera_to_target_horizontal_m']
        turn=r*(np.array([math.cos(h+math.radians(bias)),math.sin(h+math.radians(bias))])
              -np.array([math.cos(h),math.sin(h)]))
        predicted=float(np.linalg.norm(delta[:2]+turn))
        real=s['map_surface_error_p95_m']
        rows.append(dict(stamp=t,cam_body_heading_deg=bias,body_synced_bias_deg=wrap(deg(a-g)+align),
                         anchor_late_bias_deg=wrap(bias-gr_late) if gr_late is not None else None,
                         truth_timing_effect_deg=deg(gl-g),translation_xy_m=float(np.linalg.norm(delta[:2])),
                         range_m=r,predicted_horizontal_m=predicted,measured_horizontal_m=real,
                         abs_prediction_gap_m=abs(predicted-real)))
    after=[r for r in resets if cloud is not None and r['stamp']>cloud]
    gate_samples=[]
    for t in waits:
        last_good=next(((stamp,v) for stamp,v in reversed(good) if stamp<=t),None)
        last_reset=next((r['stamp'] for r in reversed(resets) if r['stamp']<=t),0.)
        stable=t-max(last_good[0],last_reset) if last_good and last_good[1] else 0.
        gate_samples.append(dict(request_sim=t,valid=bool(last_good and last_good[1]),stable_s=stable))
    states={k:transitions(flags,k) for k in ('cs_yaw_align','cs_tilt_align','cs_gps','cs_baro_hgt',
        'cs_gps_hgt','cs_mag_hdg','cs_mag_3d','cs_mag_aligned_in_flight','cs_mag_fault','reject_yaw')}
    measured={key:stats([r[key] for r in rows if r[key] is not None]) for key in rows[0] if key!='stamp'} if rows else {}
    return dict(case=case.name,ulg=str(path),ulg_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        commit=(case/'commit.txt').read_text().strip(),ekf2_mag_type=int(u.initial_parameters['EKF2_MAG_TYPE']),
        preflight_checks_pass_s=precheck,armed_s=armed,in_air_s=takeoff,heading_good_events=good,
        heading_good_first_s=ready,first_wait_vlm_s=cloud,first_capture_floor=captures[0]['floor'] if captures else None,
        heading_good_before_first_request=bool(ready and cloud and ready<cloud),late_heading_resets=after,
        gate_samples=gate_samples,pass_heading_gate=bool(gate_samples and not after and
            all(s['valid'] and s['stable_s']>=1. for s in gate_samples)),resets=resets,
        estimator_flags=states,ekf_reset_counts=transitions(es,'reset_count_quat'),
        yaw_aligned_to_imu_gps_events_s=[float(t)/1e6 for t,v in zip(ekf_events['timestamp'],ekf_events['yaw_aligned_to_imu_gps']) if v],
        ground_early_yaw_residual_deg=stats(ground_early),ground_after_px4_ready_yaw_residual_deg=stats(ground_late),
        before_midair_reset_yaw_deg=stats(air_before),after_midair_reset_yaw_deg=stats(post),
        px4_attitude_versus_mavros_yaw_deg=stats(gyro_yaw),fit_world_from_map_yaw_deg=align,
        moving_surface_p95_m=moving.get('map_surface_error_p95_of_frame_p95_m'),
        final_depth_worst_frame_p95_m=moving.get('depth_surface_max_frame_p95_m'),
        frames=rows,summary=measured)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rootfs',type=Path,required=True)
    parser.add_argument('cases',type=Path,nargs='+')
    parser.add_argument('--require-gate',action='store_true',help='Exit nonzero if any capture precedes heading readiness or sees later resets')
    args=parser.parse_args()
    cases=[audit(case,args.rootfs) for case in args.cases]
    print(json.dumps(cases,indent=2,allow_nan=False))
    if args.require_gate and not all(c['pass_heading_gate'] for c in cases):
        raise SystemExit(1)


if __name__=='__main__':
    main()
