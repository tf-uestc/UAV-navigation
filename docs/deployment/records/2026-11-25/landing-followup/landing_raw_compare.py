"""Replay archived FCU/odom timing; explicitly simulate the unrecorded ExtendedState stream."""
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tarfile
from types import SimpleNamespace
from unittest.mock import patch


def replay(trace, ground_model):
    from _support import Header, IdleThread, Point, PoseStamped, String, Time
    from test_segment_flight_safety import load_components

    code, _ = load_components()
    land = next(e for e in trace if e[1]=="flight_command" and e[2].get("command")=="LAND")
    start = land[0]
    od = sorted((t,v) for t,k,v in trace if k=="odom")
    wall_points = [t for t,_ in od]
    stamps = [v["stamp"] for _,v in od]
    def sim(t):
        j=bisect_right(wall_points,t)
        if not j or j==len(od):
            return stamps[min(max(j-1,0),len(od)-1)]
        fraction=(t-wall_points[j-1])/(wall_points[j]-wall_points[j-1])
        return stamps[j-1]+fraction*(stamps[j]-stamps[j-1])
    disarm=next(t for t,k,v in trace if t>start and k=="fcu" and not v["armed"])
    events=[(t,k,v) for t,k,v in trace if t>=start-3 and k in ("fcu","odom")]
    events.append((start,"land",land[2]))
    first_t=start-3
    final_t=od[-1][0]
    # The archived trial lacks ExtendedState callbacks. Use one distinct sample
    # per simulation second, no earlier than the recorded disarmed FCU heartbeat.
    previous=None
    for t in wall_points:
        floor=math.floor(sim(t))
        if t>=first_t and floor!=previous:
            events.append((t,"extended",floor))
        previous=floor
    for i in range(int((final_t-start)/.04)+1):
        events.append((start+i*.04,"tick",None))
    events.sort(key=lambda row:row[0])
    timebox=[first_t]
    actions=[]
    first_terminal=None
    ground_stamps=[]
    with patch("time.monotonic",side_effect=lambda:timebox[0]), patch("time.sleep"), \
         patch("threading.Thread",IdleThread), patch.object(code.FlightExecutor,"require_sitl"):
        Time.current=sim(first_t)
        controller=code.FlightExecutor()
        controller.action=lambda a:actions.append((round(timebox[0]-start,4),a))
        for t,kind,v in events:
            timebox[0]=t
            Time.current=sim(t)
            if kind=="fcu":
                controller.capture(SimpleNamespace(**v),"state")
            elif kind=="extended":
                onground=t>=disarm and ground_model=="ground_after_disarm"
                msg=SimpleNamespace(header=Header(stamp=Time.from_sec(v)),landed_state=1 if onground else 2)
                controller.capture(msg,"landed")
                if onground:
                    ground_stamps.append((round(t-start,4),v))
            elif kind=="odom":
                stamp=Time.from_sec(v["stamp"])
                stamp.is_zero=lambda:False
                pose=PoseStamped()
                pose.pose.position=Point(*v["position"])
                pose.pose.orientation.z=math.sin(v["yaw"]/2.)
                pose.pose.orientation.w=math.cos(v["yaw"]/2.)
                # Both guards use the velocity norm. The original trace saves
                # the norm but not its decomposition into xyz.
                odom=SimpleNamespace(header=Header(stamp=stamp,frame_id="map"),child_frame_id="base_link",
                    pose=pose,twist=SimpleNamespace(twist=SimpleNamespace(linear=Point(v["speed"],0,0))))
                controller.capture(odom,"odom")
            elif kind=="land":
                controller.owned=True
                controller.session=v["flight_session_id"]
                controller.bridge.mode="HOLD"
                controller.clock_advanced_at=t
                controller.command(String(json.dumps(v)))
                assert controller.bridge.mode=="LANDING"
            else:
                controller.graph,controller.graph_at=True,t
                ticks=iter((False,True))
                code.rospy.is_shutdown=lambda:next(ticks)
                controller.run()
                if controller.bridge.mode in ("COMPLETE","FAILED"):
                    first_terminal=dict(mode=controller.bridge.mode,reason=controller.bridge.reason,
                                        elapsed_wall_s=round(t-start,4),sim_s=round(Time.current,4))
                    break
    span=[v["speed"] for t,k,v in trace if k=="odom" and disarm<=t<start+30]
    return dict(land_wall_s=start,land_observed_sim_s=round(sim(start),4),
                first_disarm_wall_s=round(disarm-start,6),original_post_disarm_odom_count=len(span),
                original_post_disarm_samples_over_0p15=sum(v>.15 for v in span),
                synthetic_post_disarm_ground_samples=ground_stamps,model=ground_model,
                first_executor_terminal=first_terminal,actions=actions)


if __name__=="__main__":
    repo=Path(sys.argv[1]).resolve()
    source=Path(sys.argv[2]).resolve()
    labels=set(sys.argv[3:])
    sys.path.insert(0,str(repo/"tests/offline"))
    out={}
    with tarfile.open(source,"r|gz") as archive:
        for member in archive:
            parts=member.name.split("/")
            if len(parts)!=4 or parts[1] not in labels or parts[2:]!=["result","trace.json"]:
                continue
            raw=archive.extractfile(member).read()
            trace=json.loads(raw)
            out[parts[1]]=dict(trace_sha256=hashlib.sha256(raw).hexdigest(),
                               comparisons=[replay(trace,m) for m in ("ground_after_disarm","no_ground")])
    assert set(out)==labels
    print(json.dumps(dict(commit=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
                          original_archive=source.name,cases=out),indent=2))
