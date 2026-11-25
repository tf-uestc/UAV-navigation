"""Compare the instant of narrow-box insertion in the saved F03 and F04 trials."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import tarfile


def compare(data):
    trace=sorted(json.loads(data),key=lambda row:row[0])
    at,box=next((t,d) for t,k,d in trace if k=="obstacle_inserted")
    def nearest(kind):
        return min(((t,d) for t,k,d in trace if k==kind),key=lambda r:abs(r[0]-at))
    od_t,odom=nearest("odom")
    sp_t,sp=nearest("setpoint")
    ft,flight=nearest("flight")
    goal=flight["context"][1]
    mission=flight["context"][0]
    positions=next(d["samples"] for t,k,d in trace if k=="dynamic_box_world_track")
    x,y,_=positions[0]
    cx,cy=box["center"]
    sx,sy=box["size"][:2]
    clearance=math.hypot(max(abs(x-cx)-sx/2,0),max(abs(y-cy)-sy/2,0))
    status=next(((t,d) for t,k,d in trace if t>=at and k=="ego_status" and d.get("mission_id")==mission
                 and d["goal_id"]==goal and d["status"] in ("REPLANNED","EMERGENCY_STOP","FAILED")),None)
    mode=next((d.get("reason") for _,k,d in trace if k=="trajectory_ack" and d.get("goal_id")==goal
               and d.get("reason")=="emergency_stop"),None)
    return dict(trace_sha256=hashlib.sha256(data).hexdigest(), box_world_center=box["center"],box_size=box["size"],
        mission=mission,active_goal=goal,active_traj=flight["trajectory_id"],active_flight_mode=flight["mode"],
        insertion_followup_sim_time_s=odom["stamp"],odom_wall_offset_s=od_t-at,setpoint_wall_offset_s=sp_t-at,
        measured_speed_mps=odom["speed"],first_sample_gap_to_box_front_m=cx-sx/2-x,
        first_sample_body_lateral_minus_box_center_m=y-cy,first_sample_center_clearance_m=clearance,
        setpoint_minus_body_map_x_m=sp["position"][0]-odom["position"][0],
        initial_status=dict(wall_after_spawn_s=status[0]-at,status=status[1]["status"],reason=status[1]["reason"]) if status else None,
        accepted_emergency_for_inserted_goal=mode is not None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("archives",nargs="+",type=Path)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    rows={}
    for archive in a.archives:
        with tarfile.open(archive,"r|*") as t:
            for member in t:
                m=re.fullmatch(r"(?:\./)?flights/((?:F03-1|F04-[12])-dynamic)/result/trace.json",member.name)
                if m and member.isfile():
                    rows[m[1]]=compare(t.extractfile(member).read())
    if set(rows)!={"F03-1-dynamic","F04-1-dynamic","F04-2-dynamic"}:
        raise ValueError("All three sourced narrow-box trials are required")
    a.output.write_text(json.dumps(rows,indent=2)+"\n",encoding="utf-8")


if __name__=="__main__":
    main()
