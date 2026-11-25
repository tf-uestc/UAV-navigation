"""Read F04 ledgers, Gazebo contacts and ROS traces from local archives or directories."""

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import tarfile


COMMIT = "6b58825f8e715d6ac902303b812976ed20693e6e"
ROUTES = ("straight", "static", "long", "outside")
INITIAL = [f"F04-{i}-{s}" for i in range(1, 6) for s in ROUTES] + [
    "F04-1-dynamic", "F04-1-blocked", "F04-1-unknown"]
FOLLOWUPS = {"S04-1-offset-box": "dynamic", "S04-1-unknown": "unknown"}
LEDGERS = ("F04-attempts.jsonl", "F04-supplement-attempts.jsonl", "S04-attempts.jsonl")
LABELS = INITIAL + ["F04-6-long","F04-2-dynamic","F04-3-dynamic"] + list(FOLLOWUPS)
TARGETS = dict(straight=[("straight", "DONE")], static=[("occupied", "FAILED"), ("bypass", "DONE")],
               long=[("beyond_planning_horizon", "DONE")], outside=[("outside", "FAILED")],
               dynamic=[("dynamic", "DONE")], blocked=[("dynamic", "FAILED")],
               unknown=[("outside_view", "FAILED"), ("known_free", "DONE"), ("occupied", "FAILED"),
                        ("stale_map", "FAILED"), ("fresh_after_recovery", "DONE")])
PAIR = re.compile(rb'collision1:\s*"([^"]+)"\s+collision2:\s*"([^"]+)"')
STAMP = re.compile(rb'^time \{\s*sec:\s*(\d+)\s+nsec:\s*(\d+)\s*\}', re.M)
GROUND = {b"ground_plane", b"landing_pad"}
FILES = ("commit.txt", "source-status.txt", "result/trace.json", "result/results.json",
         "result/physics-contacts.json", "result/physics-contacts.txt", "physics-original-audit.json",
         "physics-grounded-shutdown.json", "native-delivery-original.json", "native-delivery-interpretation.json")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def raw_contacts(data):
    pairs = Counter()
    for left, right in PAIR.findall(data):
        a, b = left.split(b"::")[0], right.split(b"::")[0]
        if a == b == b"iris_depth_camera":
            pairs["iris_self"] += 1
        elif b"iris_depth_camera" in (a, b):
            other = b if a == b"iris_depth_camera" else a
            pairs["iris_" + other.decode("ascii", errors="replace")] += 1
        elif {a, b} == {b"contact_witness", b"ground_plane"}:
            pairs["positive_control"] += 1
    times = sorted(set(int(m[1]) + int(m[2])*1e-9 for m in STAMP.finditer(data)))
    # Each complete top-level stamp follows the contact pairs of that Gazebo message.
    last_frames = {}
    if times:
        recent = times[-1] - 1.001
        start = 0
        for m in STAMP.finditer(data):
            stamp = int(m[1]) + int(m[2])*1e-9
            if stamp >= recent:
                body_ground = False
                for a, b in PAIR.findall(data, start, m.start()):
                    if b.startswith(b"iris_depth_camera::"):
                        a, b = b, a
                    body_ground |= (a.startswith(b"iris_depth_camera::") and b"base_link" in a
                                    and b.split(b"::")[0] in GROUND)
                last_frames[stamp] = last_frames.get(stamp, False) or body_ground
            start = m.end()
    return dict(contact_pairs=dict(pairs), raw_sha256=digest(data), unique_frames=len(times),
                interval=[times[0], times[-1]] if times else [], _times=times, _last=last_frames)


def closest(samples, t):
    return min(samples, key=lambda row: abs(row[0]-t))[1] if samples else {}


def inspect_trace(trace, scenario):
    trace = sorted(trace, key=lambda row:row[0])
    odom = [(t,d) for t,k,d in trace if k == "odom"]
    fcu = [(t,d) for t,k,d in trace if k == "fcu"]
    armed = [t for t,d in fcu if d.get("armed") is True]
    bracket = []
    if armed and odom:
        before = [t for t,d in fcu if t < armed[0] and d.get("armed") is False]
        after = [t for t,d in fcu if t > armed[-1] and d.get("armed") is False]
        if before and after:
            walls = [t for t,_ in odom]
            li, ri = bisect_right(walls, max(before))-1, bisect_left(walls, min(after))
            if li >= 0 and 0 < ri < len(odom):
                bracket = [odom[li][1]["stamp"], odom[ri-1][1]["stamp"], odom[ri][1]["stamp"]]
    missions = [(t,d) for t,k,d in trace if k == "mission"]
    outcome = []
    for pref, required in TARGETS[scenario]:
        prefix = pref + "-"
        phases = [(t,d) for t,d in missions if str(d.get("mission_id")).startswith(prefix)]
        end = next(((t,d) for t,d in phases if d["phase"] in ("DONE","FAILED")),None)
        if not end:
            outcome.append(dict(prefix=pref, observed=None, passed=False))
            continue
        t, d = end
        pose = closest(odom, t)
        m = d["mission_id"]
        tracked = sorted({(f["context"][1], f["trajectory_id"]) for _,k,f in trace
                          if k=="flight" and f.get("mode")=="TRACK" and f.get("context") and f["context"][0]==m})
        accepted = {(a["goal_id"],a["traj_id"]) for _,k,a in trace if k=="trajectory_ack"
                    and a.get("mission_id")==m and a.get("accepted") is True}
        plans = [p for _,k,p in trace if k=="plan_request" and p.get("mission_id")==m]
        holds = [f for qt,k,f in trace if k=="flight" and t-.3<=qt<=t+.7 and f.get("mode")=="HOLD"
                 and f.get("context") is None and f.get("stop_confirmed") is True]
        err = math.dist(pose["position"], d["final_goal"]) if pose and d.get("final_goal") else None
        flags = dict(expected_terminal=d["phase"]==required, only_accepted_tracks=set(tracked)<=accepted)
        if required == "DONE":
            flags.update(actually_tracked=bool(tracked), endpoint=err is not None and err<=.23 and pose["speed"]<=.15)
        else:
            if pref in ("outside","outside_view","occupied","stale_map"):
                flags["never_tracked"] = not tracked
            if pref in ("outside","stale_map"):
                flags["no_planner_request"] = not plans
            if pref=="outside":
                flags["outside_fence_reason"] = d["reason"]=="outside_fence"
            if pref=="outside_view":
                flags["observe_once_unknown"] = d["reason"]=="unknown_trajectory" and sum(v["phase"]=="OBSERVE_MAP" for _,v in phases)==1
            if scenario=="blocked" or pref in ("outside","occupied") and scenario!="unknown":
                flags["confirmed_stop"] = bool(holds)
        if pref=="beyond_planning_horizon":
            first = closest(odom,phases[0][0])
            flags["crosses_local_horizon"] = bool(first and d.get("final_goal")
                and math.dist(first["position"],d["final_goal"])>7.5 and d["segment_count"]>=5)
        outcome.append(dict(mission=m, observed=d["phase"], reason=d["reason"], goal_error_m=err,
                            nearest_odom_speed_mps=pose.get("speed"), segments=d.get("segment_count"),
                            planned_requests=len(plans), tracked=tracked, checks=flags,passed=all(flags.values())))
    bounds = None
    takeoff = next((t for t,d in missions if "takeoff" in str(d.get("mission_id")) and d["phase"]=="DONE"),None)
    landing = next((t for t,d in missions if takeoff is not None and t>takeoff and d["phase"]=="LANDING"),None)
    if takeoff is not None and landing is not None:
        points = [d["position"] for t,d in odom if takeoff <= t <= landing]
        limits = ((-6,6),(-4,4),(1.1,2.35))
        bounds = dict(count=len(points), outside=sum(any(not lo<=v<=hi for v,(lo,hi) in zip(p,limits)) for p in points))
    obstacle = next(((t,d) for t,k,d in trace if k=="obstacle_inserted"),None)
    path = next((d for t,k,d in trace if k in ("world_obstacle_track","dynamic_box_world_track")),None)
    clearance = None
    if path and path["samples"]:
        fixture = obstacle[1] if obstacle else path
        cx,cy=fixture["center"]; sx,sy=fixture["size"][:2]
        points = path["samples"]
        minimum=min(math.hypot(max(0,abs(x-cx)-sx/2),max(0,abs(y-cy)-sy/2)) for x,y,_ in points)
        clearance=dict(samples=len(points), center_to_surface_m=minimum,
                       stayed_before_wall=max(p[0] for p in points)<cx-sx/2,
                       finished_beyond_box=points[-1][0] > cx+.35)
    deliveries = None
    if obstacle:
        t0=obstacle[0]
        dyn=next((d["mission_id"] for _,d in missions if str(d.get("mission_id")).startswith("dynamic-")),None)
        packets={}
        for t,k,d in trace:
            if k not in ("native_spline","forwarded_spline","trajectory_ack","gateway_disposition") or d.get("mission_id")!=dyn:
                continue
            key=(d["planner_session_id"],d["goal_id"],d["traj_id"])
            rec=packets.setdefault(key,{})
            rec[k]=dict(offset_wall_s=round(t-t0,4),reason=d.get("reason"),emergency=d.get("emergency"),accepted=d.get("accepted"))
        stops=[(t,d) for t,k,d in trace if t>=t0 and k=="flight" and d.get("mode")=="HOLD"
               and d.get("stop_confirmed") is True and d.get("context") is None]
        status=[]
        for t,k,d in trace:
            if t<t0 or k!="ego_status" or d.get("mission_id")!=dyn or d.get("status")=="READY":
                continue
            tag=(d["status"],d["reason"],d["goal_id"])
            delta=round(t-t0,4)
            if status and tag==(status[-1]["status"],status[-1]["reason"],status[-1]["goal"]):
                status[-1]["observations"]+=1
                status[-1]["last_wall_s"]=delta
            else:
                status.append(dict(offset_wall_s=delta,last_wall_s=delta,observations=1,
                                   status=tag[0],reason=tag[1],goal=tag[2]))
        warnings=[q for q in status if q["status"]=="EMERGENCY_STOP" and q["reason"]=="collision"]
        warning_packets=[]
        for warning in warnings:
            for (session, g, n), events in packets.items():
                raw=events.get("native_spline",{})
                if g==warning["goal"] and raw.get("emergency") and raw["offset_wall_s"]>=0:
                    warning_packets.append(dict(goal=g, trajectory=n, planner_session=session,
                        warning_s=warning["offset_wall_s"], native_s=raw["offset_wall_s"],
                        executor_accepted=events.get("trajectory_ack",{}).get("accepted"),
                        gateway_rejection=events.get("gateway_disposition",{}).get("reason")))
        phases=[(t,d) for t,d in missions if t>=t0 and d.get("mission_id")==dyn]
        detours=[]
        for brake, state in phases:
            if state["phase"]!="SETTLE" or state.get("reason") not in (
                    "planner_stop","occupied_trajectory","unknown_trajectory","outside_known_map"):
                continue
            resume=next(((t,d) for t,d in phases if t>brake and d["phase"]=="TRACK"
                         and d.get("detour_active") and (d.get("goal_id") or 0)>(state.get("goal_id") or 0)),None)
            if resume:
                h=next((t for t,_ in stops if brake<=t<resume[0]),None)
                detours.append(dict(braked_s=brake-t0, guard_reason=state["reason"],
                    confirmed_hold_s=h-t0 if h is not None else None,
                    hold_speed_mps=closest(odom,h).get("speed") if h is not None else None,
                    resumed_s=resume[0]-t0, new_goal=resume[1]["goal_id"]))
        deliveries=dict(mission=dyn, after_insertion_status=status,
                        first_measured_hold=(dict(offset_wall_s=stops[0][0]-t0, speed_mps=closest(odom,stops[0][0])["speed"]) if stops else None),
                        guarded_resume_sequences=detours, matching_collision_stop_packets=warning_packets,
                        packets=[dict(session=k[0],goal=k[1],traj=k[2],**v) for k,v in sorted(packets.items())])
    unknown=None
    if scenario=="unknown":
        fresh=next((d["mission_id"] for _,d in missions if str(d.get("mission_id")).startswith("fresh_after_recovery-")),None)
        seq=[(t,d["phase"],d.get("reason")) for t,d in missions if fresh and d.get("mission_id")==fresh]
        done=next((t for t,phase,_ in seq if phase=="DONE"),None)
        lands=[(t,d) for t,k,d in trace if k=="flight_command" and d.get("command")=="LAND"]
        later_controls=[(k,d.get("context"),d.get("trajectory_id")) for t,k,d in trace if done is not None and t>done and
                        ((k=="flight" and d.get("mode")=="TRACK") or k=="plan_request" or
                         (k=="flight_command" and d.get("command") in ("TRACK","PLAN")))]
        stale=next((t for t,d in missions if str(d.get("mission_id")).startswith("stale_map-") and d.get("phase")=="FAILED"),None)
        fresh_start=seq[0][0] if seq else None
        revived=[d.get("context") for t,k,d in trace if stale is not None and fresh_start is not None
                 and stale<t<fresh_start and k=="flight" and (d.get("mode")=="TRACK" or d.get("context") is not None)]
        unknown=dict(fresh_mission=fresh, fresh_phase_order=seq, land_commands=[t for t,_ in lands],
                     done_before_first_land=done is not None and bool(lands) and done<lands[0][0],
                     old_contexts_between_stale_and_fresh=revived, old_track_or_plan_after_done=later_controls,
                     first_disarmed_after_landing=next((t for t,d in fcu if lands and t>=lands[0][0] and not d["armed"]),None))
    return dict(missions=outcome, fence=bounds, obstacle=clearance, delivery=deliveries,
                unknown_recovery=unknown, disarm_bracket_sim_s=bracket,
                last_connected_disarmed=bool(fcu and fcu[-1][1].get("connected") and fcu[-1][1].get("armed") is False))


def slot(name):
    name=name.removeprefix("./") if hasattr(name,"removeprefix") else name.lstrip("./")
    if name in LEDGERS:
        return "ledgers", name
    for label in LABELS:
        if name in (label+"-ground.json", label+"-audit.json"):
            return label, name[len(label)+1:]
        prefix="flights/"+label+"/"
        if name.startswith(prefix) and name[len(prefix):] in FILES:
            return label, name[len(prefix):]
    return None


def consume(collection, name, data):
    label, key = slot(name)
    group=collection[label]
    if key in group:
        raise ValueError("Duplicate archive entry "+name)
    if key.endswith(".jsonl"):
        val=[json.loads(line) for line in data.splitlines() if line.strip()]
        group[key]=[dict(event=r["event"], label=r.get("label"),passed=r.get("passed"),exit_code=r.get("exit_code"))
                    for r in val if r.get("event") in ("start","finish")]
    elif key=="result/physics-contacts.txt":
        group[key]=raw_contacts(data)
    elif key=="result/trace.json":
        group[key]=inspect_trace(json.loads(data),FOLLOWUPS.get(label,label.split("-")[-1]))
        group[key]["sha256"]=digest(data)
    elif key.endswith(".json"):
        group[key]=json.loads(data)
    else:
        group[key]=data.decode().strip()


def inspect_case(label, group, ledger):
    s=FOLLOWUPS.get(label,label.split("-")[-1])
    tr=group["result/trace.json"]
    c=group["result/physics-contacts.txt"].copy()
    frames=c.pop("_times"); tail=c.pop("_last")
    primary=group["result/results.json"]
    contact=group["result/physics-contacts.json"]
    original=group["physics-original-audit.json"]["physics_coverage"]
    br=tr["disarm_bracket_sim_s"]
    span, maxgap=False,None
    if br and frames:
        start, end=br[0],br[-1]
        span=frames[0]<=start<=end<=frames[-1]
        a=max(0,bisect_right(frames,start)-1); b=bisect_left(frames,end)
        covered=frames[a:b+1]
        if len(covered)>1:
            maxgap=max(y-x for x,y in zip(covered,covered[1:]))
    stop=contact.get("end_stamp")
    recent=[(t,b) for t,b in tail.items() if stop is not None and stop-1.000001<=t<=stop+.000001]
    supplemental = group.get("physics-grounded-shutdown.json",{}).get("passed") is True
    resolved = bool(supplemental and br and frames and stop is not None and br[1]<=stop<br[2]
                    and abs(frames[-1]-stop)<1e-8 and len(recent)>=100 and all(b for _,b in recent))
    counts=c["contact_pairs"]
    unplanned={k:v for k,v in counts.items() if k.startswith("iris_") and k not in ("iris_self","iris_landing_pad","iris_ground_plane")}
    physical=(span or resolved) and maxgap is not None and maxgap<1 and not unplanned and counts.get("positive_control",0)>0
    c.update(unexpected=unplanned, conservative_full_span=span, independent_resolved_ground_shutdown=resolved,
             maximum_covered_gap_s=maxgap, confirmed_body_ground_frames_in_recorder_last_second=sum(b for _,b in recent),
             frames_in_recorder_last_second=len(recent), last_physics_frame_body_ground=tail.get(frames[-1]) if frames else None,
             summary_positive_count_agrees=counts.get("positive_control",0)==contact["positive_control_contacts"],
             F04_coverage_agrees=(original.get("passed") or supplemental)==bool(physical))
    g=group["ground.json"].get("samples",[])
    ground=len(g)>=2 and all(row.get("state",{}).get("connected") is True and row["state"].get("landed") is True
        and row["state"].get("armed") is False for row in g[-2:])
    expected=all(m["passed"] for m in tr["missions"])
    flags=dict(terminal_and_tracking=expected, fenced=bool(tr["fence"] and tr["fence"]["count"] and not tr["fence"]["outside"]),
               physical_flight_coverage=bool(physical), ground_samples=ground, last_fcu_disarmed=tr["last_connected_disarmed"],
               single_frozen_source=group.get("commit.txt")==COMMIT and group.get("source-status.txt")=="",
               positive_control_summary_matches=c["summary_positive_count_agrees"])
    if s in ("dynamic","static","blocked"):
        o=tr["obstacle"]
        flags["obstacle_outcome"]=bool(o and o["center_to_surface_m"]>.24 and (o["stayed_before_wall"] if s=="blocked" else o["finished_beyond_box"]))
    if s=="unknown":
        u=tr["unknown_recovery"]
        flags["recovery_then_land_no_revival"]=bool(u["done_before_first_land"] and not u["old_contexts_between_stale_and_fresh"]
            and not u["old_track_or_plan_after_done"] and u["first_disarmed_after_landing"] is not None)
    d=tr["delivery"]
    if d:
        obs=[]
        for p in d["packets"]:
            valid=("forwarded_spline" in p and "trajectory_ack" in p and "gateway_disposition" not in p)
            retired=("forwarded_spline" not in p and "trajectory_ack" not in p and
                     p.get("gateway_disposition",{}).get("accepted") is False and p.get("native_spline",{}).get("emergency") is True)
            obs.append("native_spline" in p and (valid or retired))
        flags["native_dispositions"]=bool(obs) and all(obs)
        if s=="dynamic":
            mode=next((x.get("mode") for x in primary["checks"] if x["name"]=="dynamic_replan_executed"),None)
            if mode in ("guarded_detour","settled_detour"):
                flags["measured_hold_then_guarded_resume"]=any(h["confirmed_hold_s"] is not None
                    and h["hold_speed_mps"]<=.15 for h in d["guarded_resume_sequences"])
            elif mode=="native_continuous":
                replans={q["goal"] for q in d["after_insertion_status"] if q["status"]=="REPLANNED" and q["reason"]=="collision"}
                flags["continuous_replan_executed"]=any(g in replans and n>1 for m in tr["missions"]
                    for g,n in m.get("tracked",[]))
            else:
                flags["replan_mode_measured"]=False
    run=[r for r in ledger if r.get("label")==label]
    if len(run)!=2 or [r["event"] for r in run]!=["start","finish"]:
        raise ValueError("Incomplete or duplicate ledger events: "+label)
    event=run[-1]
    behavior=all(flags.values())
    if label == "S04-1-unknown":
        order=tr["unknown_recovery"]["fresh_phase_order"]
        done=next((t for t,phase,reason in order if phase=="DONE" and reason=="destination_reached"),None)
        flags["mission_landed_recorded_after_destination"]=done is not None and any(t>done and phase=="LANDED" for t,phase,_ in order)
    if d:
        warning=any(q["status"]=="EMERGENCY_STOP" and q["reason"]=="collision" for q in d["after_insertion_status"])
        emergency=any(p.get("native_spline",{}).get("emergency") is True for p in d["packets"])
        continuous=s=="dynamic" and flags.get("continuous_replan_executed") is True and not warning and not emergency
        flags["required_emergency_evidence_or_continuous_bypass"]=bool(d["matching_collision_stop_packets"] or continuous)
    passed=all(flags.values())
    return dict(label=label, original=label in INITIAL, scenario=s, F04_tool_passed=event["passed"],
                F04_tool_failed=group["audit.json"].get("failed"), independent=passed, agreement=passed==event["passed"],
                behavior_and_physics_passed=behavior,
                flags=flags, primary_passed=primary.get("passed"), terminal_disarmed=primary.get("final_armed") is False,
                observer_exit=event.get("exit_code"), contacts=c, trace=tr,
                frozen_native_delivery=group.get("native-delivery-interpretation.json"),
                runtime_checks=[x["name"] for x in primary.get("checks",[])])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sources",type=Path,nargs="+",help="archives or extracted experiment directories")
    parser.add_argument("--output",type=Path,help="save JSON to this file; defaults to stdout")
    args=parser.parse_args()
    groups=defaultdict(dict)
    for path in args.sources:
        if path.is_dir():
            candidates=[path / n for n in LEDGERS]
            for label in LABELS:
                candidates += [path/(label+"-ground.json"),path/(label+"-audit.json")]
                candidates += [path/"flights"/label/n for n in FILES]
            for p in candidates:
                if p.is_file():
                    consume(groups,p.relative_to(path).as_posix(),p.read_bytes())
        else:
            with tarfile.open(path,"r|*") as archive:
                for member in archive:
                    if member.isfile() and slot(member.name):
                        consume(groups,member.name,archive.extractfile(member).read())
    ledger=[event for rows in groups["ledgers"].values() for event in rows]
    started=[r["label"] for r in ledger if r["event"]=="start"]
    if len(set(started))!=len(started) or any(label not in LABELS for label in started):
        raise ValueError("Unexpected or duplicate trial identifier")
    if "F04-attempts.jsonl" in groups["ledgers"] and set(started)&set(INITIAL)!=set(INITIAL):
        raise ValueError("Incomplete initial 23-trial archive")
    if not started or any(label not in started for label in groups if label!="ledgers"):
        raise ValueError("No trial ledger, or trial evidence missing from ledger")
    rows=[inspect_case(label,groups[label],ledger) for label in started]
    output=json.dumps(dict(commit=COMMIT, original_count=sum(label in INITIAL for label in started), trials=rows),indent=2)+"\n"
    if args.output:
        args.output.write_text(output,encoding="utf-8")
    else:
        sys.stdout.write(output)


if __name__=="__main__":
    main()
