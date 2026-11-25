#!/usr/bin/env python3
"""Second-pass read-only grading of completed SITL trials, including raw physics coverage."""
import argparse
from bisect import bisect_left, bisect_right
import hashlib
import json
import mmap
from pathlib import Path
import re
import sys

from crosscheck_planning import grade


STAMP = re.compile(rb'^time \{\s*sec: (\d+)\s*nsec: (\d+)\s*\}', re.M)
PAIR = re.compile(rb'collision1:\s*"([^"]+)"\s+collision2:\s*"([^"]+)"')


def contact_report(folder):
    result = folder/'result'
    summary = json.loads((result/'physics-contacts.json').read_text())
    trace = json.loads((result/'trace.json').read_text())
    fcu = sorted((t,d['armed']) for t,k,d in trace if k=='fcu')
    armed = [t for t,flag in fcu if flag]
    pre_arm = max(t for t,flag in fcu if not flag and t<min(armed))
    post_disarm = min(t for t,flag in fcu if not flag and t>max(armed))
    odom = sorted((t,d['stamp']) for t,k,d in trace if k=='odom')
    walls = [t for t,_ in odom]
    left = max(0,bisect_right(walls,pre_arm)-1)
    right = bisect_left(walls,post_disarm)
    begin = odom[left][1]
    end = odom[right][1]
    with (result/'physics-contacts.txt').open('rb') as stream, mmap.mmap(
            stream.fileno(),0,access=mmap.ACCESS_READ) as raw:
        times = [int(s)+int(ns)*1e-9 for s,ns in STAMP.findall(raw)]
        positive = 0
        obstacles = []
        for a,b in PAIR.findall(raw):
            positive += (b'contact_witness' in a+b and b'ground' in a+b)
            if (b'iris_depth_camera::' in a)==(b'iris_depth_camera::' in b):
                continue
            other = b if b'iris_depth_camera::' in a else a
            if not any(x in other.split(b'::') for x in (b'ground_plane', b'landing_pad')):
                obstacles.append(other.decode('utf8','replace'))
    if not times:
        raise ValueError('No complete Gazebo top-level time envelope')
    # Use the absolute simulation timeline to bracket every armed interval. A
    # queued physics frame may legitimately be written after observer shutdown
    # was requested. Its observation time remains valid.
    reversals=[x-y for x,y in zip(times,times[1:]) if y<x]
    # The transport can serialize neighboring frames in a different order.
    # Check for a simulator clock reset using odometry on its own timeline,
    # and measure observation coverage using the frames' acquisition stamps.
    times=sorted(set(times))
    a=max(0,bisect_right(times,begin)-1)
    b=bisect_left(times,end)
    during=times[a:b+1]
    gaps=[y-x for x,y in zip(during,during[1:])]
    max_gap=max(gaps,default=float('inf'))
    checks=dict(odom_clock_nondecreasing=all(x[1]<=y[1] for x,y in zip(odom,odom[1:])),
                brackets_entire_flight=times[0]<=begin<=end<=times[-1],
                no_gap_of_one_second=max_gap<1.,
                count=len(during)>=100,
                raw_agrees_with_summary=abs(times[-1]-summary['last_message_stamp'])<1e-6,
                within_one_second_at_shutdown=abs(summary['end_stamp']-times[-1])<1.,
                positive_control=positive>0 and positive==summary['positive_control_contacts'],
                zero_unexpected_contacts=not obstacles and not summary['iris_obstacle_contacts'],
                primary_contact_check=summary['passed'])
    return dict(passed=all(checks.values()),checks=checks,raw_physics_interval=[times[0],times[-1]],
                armed_flight_conservative_interval=[begin,end],top_level_messages=len(times),
                flight_messages=len(during),largest_gap_during_flight_s=max_gap,
                arrival_order_reversals=len(reversals),max_arrival_order_reversal_s=max(reversals,default=0.),
                stop_requested_sim=summary['end_stamp'],positive_control_pairs=positive,
                unexpected_contact_count=len(obstacles))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('flight_root',type=Path)
    parser.add_argument('--out',required=True,type=Path)
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    grader_hash=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    base_hash=hashlib.sha256(Path(__file__).with_name('crosscheck_planning.py').read_bytes()).hexdigest()
    for folder in sorted(args.flight_root.glob('F03-*')):
        if not ((folder/'flight.exit').is_file() or (folder/'independent-planning.json').is_file()):
            continue
        out=args.out/(folder.name+'.json')
        if out.is_file():
            continue
        try:
            report=grade(folder,folder.name.split('-')[-1])
            contact=contact_report(folder)
            report['checks']['contact_stream_observed']=contact['passed']
            report['physics_coverage']=contact
            report['passed']=all(report['checks'].values())
        except (ValueError,IndexError,KeyError,StopIteration,OSError) as exc:
            report=dict(passed=False,evidence_error=repr(exc))
        report['grader_sha256']=grader_hash
        report['base_grader_sha256']=base_hash
        out.write_text(json.dumps(report,indent=2)+'\n')
        print(folder.name,report['passed'],[k for k,v in report.get('checks',{}).items() if not v],
              report.get('evidence_error',''),flush=True)


if __name__=='__main__':
    main()
