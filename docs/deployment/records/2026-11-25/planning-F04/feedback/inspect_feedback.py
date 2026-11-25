"""Read the observer traces and simulator child logs. Emits JSON to stdout."""
import hashlib
import itertools
import json
import statistics
import sys
from pathlib import Path


def rounded(number):
    return round(number, 6) if isinstance(number, float) else number


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cadence(samples, reference):
    if len(samples) < 2:
        return {"n": len(samples)}
    pairs = list(zip(samples, samples[1:]))
    gaps = [b[0] - a[0] for a, b in pairs]
    a, b = pairs[gaps.index(max(gaps))]
    return {"n": len(samples), "median_Hz": rounded(1 / statistics.median(gaps)),
            "max_wall_gap_s": rounded(b[0] - a[0]),
            "max_gap_recv_rel_s": [rounded(t - reference) for t, _, _ in (a, b)],
            "header_stamps_at_gap": [v.get("stamp") for _, _, v in (a, b)]}


def inspect(directory):
    trace_path = directory / "result/trace.json"
    if not trace_path.is_file():
        return {"id": directory.name, "final_trace_available": False}
    # Sort reception clock for independent subscriber threads.
    events = sorted(json.loads(trace_path.read_text()), key=lambda row: row[0])
    mission = next((v["mission_id"] for _,k,v in events if k == "mission" and
                    (v.get("mission_id") or "").startswith("beyond_planning_horizon-")), None)
    if not mission:
        return {"id": directory.name, "final_trace_available": False}
    stage = [e for e in events if e[1]=="mission" and e[2].get("mission_id")==mission
             and e[2].get("goal_id")==5]
    start = next(e[0] for e in stage if e[2]["phase"]=="TRACK")
    brake = next(e[0] for e in stage if e[2]["phase"]=="BRAKE")
    terminal = next(e for e in stage if e[2]["phase"] in ("DONE", "FAILED"))
    ref = terminal[0]
    window = [e for e in events if brake-4 <= e[0] <= brake+.18]
    streams = {k:cadence([e for e in window if e[1]==k], ref)
               for k in ("odom","px4_target","setpoint","flight","fcu")}
    streams["control_status_ack"] = cadence(
        [e for e in window if e[1]=="flight_command" and e[2].get("command")=="ACK_STATUS"], ref)
    start_info = json.loads((directory / "start.json").read_text())
    simulator_log = Path(start_info["owner"]["log"])
    px4_log = simulator_log.with_name("px4.log")
    checks_path = directory / "physics-original-audit.json"
    physics = json.loads(checks_path.read_text()) if checks_path.is_file() else {}
    independent = json.loads((directory/"independent-planning.json").read_text())
    output = {
        "id": directory.name, "commit":(directory/"commit.txt").read_text().strip(),
        "mission":mission, "goal_id":5, "mission_terminal":terminal[2]["phase"],
        "mission_reason":terminal[2]["reason"], "terminal_wall_monotonic":ref,
        "track_to_first_brake_wall_s":rounded(brake-start),
        "streams_brake_minus_4_to_plus_0p18":streams,
        "subscriber_cadence_wall_time_base":"Observer receive time, measured per channel",
        "sha256":{"trace.json":digest(trace_path),"results.json":digest(directory/"result/results.json"),
                  "supervisor.log":digest(simulator_log),"px4.log":digest(px4_log)},
        "supervisor": str(simulator_log),
        "px4_outcomes":[line.strip() for line in px4_log.read_text().splitlines()
                        if any(term in line for term in ("Landing detected","Disarmed by landing",
                                                          "Opened full log file"))],
        "postflight":json.loads((directory / "postflight-state.json").read_text())["state"],
        "independent_planning":{"passed": independent["passed"], "details": independent.get("details")},
        "physics":{"unexpected":physics.get("physics_coverage",{}).get("unexpected_contact_count"),
                   "validated":physics.get("physics_coverage",{}).get("passed")},
        "late_track_or_authorize_count":sum(1 for t,k,v in events if t>=ref and k=="flight"
                                            and (v.get("mode")=="TRACK" or v.get("context") is not None)),
    }
    if terminal[2]["phase"]!="FAILED":
        return output
    t0=streams["odom"]["max_gap_recv_rel_s"][0]
    setpoints=[(t,v) for t,k,v in window if k=="setpoint"]
    frozen=max((list(group) for _,group in itertools.groupby(setpoints, lambda e:e[1].get("stamp"))),
               key=lambda group:group[-1][0]-group[0][0])
    stamp=frozen[0][1]["stamp"]
    output["repeated_setpoints"]={"stamp":stamp,"count":len(frozen),
        "wall_span_s":rounded(frozen[-1][0]-frozen[0][0]),
        "distinct_positions":len({tuple(v["position"]) for _,v in frozen})}
    output["odom_observed_500ms_limit_crossing_rel_s"]=rounded(t0+.5)
    fields=("phase","mode","command","seq","reason","status","traj_id","trajectory_id","emergency",
            "context","stop_id","stop_confirmed","owner","owns_flight","ready","accepted","armed","connected")
    timeline=[]
    last_flight=None
    last_fcu=None
    for t,k,v in events:
        if t < ref-.7:
            continue
        if k=="flight":
            state=(v.get("mode"),v.get("seq"),v.get("trajectory_id"),v.get("owns_flight"))
            if state==last_flight:
                continue
            last_flight=state
        elif k=="fcu":
            if last_fcu==v:
                continue
            last_fcu=v
        elif k=="flight_command" and v.get("command")=="ACK_STATUS":
            continue
        elif k not in ("native_spline","gateway_disposition","trajectory_ack","forwarded_spline",
                       "mission","flight_command","plan_result"):
            continue
        timeline.append(dict(rel_s=rounded(t-ref),topic=k,**{key:v[key] for key in fields if key in v}))
    output["observed_event_transitions"]=timeline
    return output


if __name__=="__main__":
    root = Path(sys.argv[1])
    print(json.dumps([inspect(root/"flights"/name) for name in sys.argv[2:]], indent=2))
