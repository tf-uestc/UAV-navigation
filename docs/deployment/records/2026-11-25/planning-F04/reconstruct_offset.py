"""Reconstruct the accepted SITL spline at the offset box from the archived Gazebo fixture."""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import tarfile


def spline_point(points, knots, order, t):
    if len(knots) != len(points) + order + 1 or not knots[order] <= t <= knots[len(points)]:
        raise ValueError("Invalid spline domain")
    i = min(len(points) - 1, bisect_right(knots, t) - 1)
    d = [list(points[j]) for j in range(i - order, i + 1)]
    for r in range(1, order + 1):
        for j in range(order, r - 1, -1):
            left = i - order + j
            span = knots[i + 1 + j - r] - knots[left]
            alpha = (t - knots[left]) / span if span else 0
            d[j] = [(1-alpha)*a + alpha*b for a, b in zip(d[j-1], d[j])]
    return d[order]


def reconstruct(raw):
    trace = json.loads(raw)
    inserted, fixture = next((t,d) for t,k,d in trace if k == 'obstacle_inserted')
    frame = next(d for t,k,d in trace if k == 'offset_frame_reference')
    before = [(k,d) for t,k,d in sorted(trace,key=lambda row:row[0]) if t <= inserted]
    active = next(d for k,d in reversed(before) if k == 'flight')
    if active['mode'] != 'TRACK' or active['trajectory_id'] != fixture['first_trajectory_id']:
        raise ValueError('No matching active trajectory at insertion')
    mid, gid, sid = active['context']
    key = (mid, gid, sid, active['trajectory_id'])
    latest = {k:d for k,d in before if k in ('trajectory_ack','forwarded_spline','native_spline') and
              (d['mission_id'],d['goal_id'],d['planner_session_id'],d['traj_id']) == key}
    if len(latest) != 3 or latest['trajectory_ack']['accepted'] is not True:
        raise ValueError('Active spline lacks matching native, forwarded or acknowledged packet')
    curve = latest['native_spline']
    order, points, knots = curve['order'], curve['points'], curve['knots']
    cx, cy = fixture['center']
    sx, sy, sz = fixture['size']
    world, wy = frame['world']
    odom, oy = frame['odom']
    yaw = wy - oy
    def world_point(t, angle):
        p = spline_point(points,knots,order,t)
        dx,dy = p[0]-odom[0],p[1]-odom[1]
        return (world[0]+math.cos(angle)*dx-math.sin(angle)*dy,
                world[1]+math.sin(angle)*dx+math.cos(angle)*dy, world[2]+p[2]-odom[2])
    sp = next(d for k,d in reversed(before) if k == 'setpoint')
    od = next(d for k,d in reversed(before) if k == 'odom')
    sample = sp['stamp'] - curve['start_time_ns']*1e-9
    if not knots[order] <= sample < knots[-order-1]:
        raise ValueError('Last setpoint outside spline interval')
    def x_crossing(plane, angle):
        ts = [sample + (knots[-order-1]-sample)*i/200 for i in range(201)]
        brackets = [(a,b) for a,b in zip(ts,ts[1:]) if world_point(a,angle)[0] <= plane < world_point(b,angle)[0]]
        if len(brackets) != 1:
            raise ValueError('Cannot isolate one forward intersection through box plane')
        a,b = brackets[0]
        for _ in range(46):
            m = (a+b)/2
            if world_point(m,angle)[0] < plane:
                a = m
            else:
                b = m
        t = (a+b)/2
        p = world_point(t,angle)
        return dict(x_plane_m=plane, spline_time_s=t, after_last_setpoint_sim_s=t-sample,
                    world_xyz_m=p, center_lateral_offset_m=cy-p[1],
                    lateral_edge_gap_m=max(0,abs(p[1]-cy)-sy/2), within_box_height=0<=p[2]<=sz)
    cuts = [x_crossing(cx+dx,yaw) for dx in (-sx/2,0,sx/2)]
    setpoint_err = math.dist(spline_point(points,knots,order,sample),sp['position'])
    return dict(trace_sha256=hashlib.sha256(raw).hexdigest(), active=dict(mission=mid,goal=gid,session=sid,trajectory=key[-1]),
                order=order, control_points=len(points),knots=len(knots), start_sim_s=curve['start_time_ns']*1e-9,
                frame_world_xyz_yaw=frame['world'],frame_odom_xyz_yaw=frame['odom'],
                frame_observed_sim_s=frame['observed_sim'], frame_odom_sim_s=frame['odom_stamp'],
                frame_observation_offset_s=frame['observed_sim']-frame['odom_stamp'],world_minus_odom_yaw_rad=yaw,
                box_center_xy_m=fixture['center'],box_size_xyz_m=fixture['size'],
                intended_y_shift_from_measured_world_origin_m=cy-world[1],setpoint_sample_sim_s=sp['stamp'],
                setpoint_xyz_odom_m=sp['position'], last_odom_speed_before_insertion_mps=od['speed'],
                last_setpoint_reconstruction_difference_m=setpoint_err,
                nominal_inflation_m=.349, planar_transform_cuts=cuts,
                translation_only_center_crossing=x_crossing(cx,0),
                within_nominal_inflation_at_center=cuts[1]['within_box_height'] and cuts[1]['lateral_edge_gap_m'] < .349)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    member='flights/S04-1-offset-box/result/trace.json'
    with tarfile.open(args.archive,'r|*') as f:
        raw=next(f.extractfile(m).read() for m in f if m.isfile() and m.name==member)
    args.output.write_text(json.dumps(reconstruct(raw),indent=2)+'\n', encoding='utf-8')


if __name__=='__main__':
    main()

