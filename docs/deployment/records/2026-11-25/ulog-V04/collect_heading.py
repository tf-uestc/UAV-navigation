"""Offline-only cross-check of cold-start PX4 ULogs against observed capture times."""
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path("/root/autodl-tmp/uav/validation/goal-20261125")
ROOTFS = ROOT.parent / "u11-clean-2LR48q/PX4-Autopilot/build/px4_sitl_default/rootfs"
COMMIT = "6b58825f8e715d6ac902303b812976ed20693e6e"
EXPECTED = ["V04-%s-%s-%d" % (distance, direction, repeat)
            for distance, directions in (("2.4", ("front", "left45", "right45", "left90", "right90")),
                                         ("1.5", ("front", "left45", "right45")))
            for direction in directions for repeat in (1, 2)]
LOG_NAME = re.compile(r"\.\/log\/2026-11-25\/(\d{2}_\d{2}_\d{2}\.ulg)")


def require(ok, detail):
    if not ok:
        raise ValueError(detail)


def regular(path):
    require(path.is_absolute() and path.resolve(strict=True) == path and path.is_file(),
            "Unsafe evidence path: " + str(path))
    return path


def finite(value):
    value=float(value)
    return value if math.isfinite(value) else None


def read(path):
    return json.loads(regular(path).read_text(encoding="utf8"))


def sha(path):
    h = hashlib.sha256()
    with regular(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def wait_times(trace):
    states = [v for _, kind, v in trace if kind == "mission" and v.get("mission_id")]
    captures = {(v["mission_id"], v["request_id"]): v for _, kind, v in trace if kind == "search_capture"}
    results = {(v["mission_id"], v["request_id"]): v for _, kind, v in trace if kind == "detect_result"}
    requests = []
    for state in states:
        if state.get("phase") != "WAIT_VLM":
            continue
        key = state["mission_id"], state["detection_request_id"]
        item = captures.get(key, {})
        result = results.get(key, {})
        requests.append(dict(id=key[1], mission=key[0], request_s=state["observed_sim"],
                             offset_deg=state.get("scan_offset_deg"), floor=item.get("floor"),
                             stop_confirmed=item.get("stop_confirmed"), frame_s=result.get("capture_stamp")))
    return states, requests


def gate_stream(publish, sample, good, counter, heading_delta):
    require(len(publish) == len(sample) == len(good) == len(counter) == len(heading_delta) > 1,
            "Empty or inconsistent position stream")
    require(all(a <= b for a, b in zip(publish, publish[1:])), "Reversed publish clock")
    since = None
    labels = []
    resets = []
    for i, (ts, valid, rc) in enumerate(zip(sample, good, counter)):
        if i and counter[i-1] != rc:
            resets.append(dict(stamp=publish[i], sample_s=ts, counter=int(rc),
                               delta_deg=math.degrees(heading_delta[i])))
        if not valid:
            since = None
        elif (since is None or rc != counter[i-1] or ts < sample[i-1]
              or round((ts-sample[i-1])*1e6) > 650000):
            since = ts
        labels.append(since)
    return labels, resets


def gate_at(t, publish, sample, good, counter, since):
    if t is None or not math.isfinite(t) or t <= 0:
        return None
    i = bisect_right(publish, t) - 1
    if i < 0:
        return dict(pass_gate=False, unavailable=True)
    age, run = t-sample[i], (sample[i]-since[i] if since[i] is not None else 0.)
    # ULog clock values are integer microseconds; enforce exact threshold boundaries.
    return dict(pass_gate=bool(good[i] and -20000 <= round(age*1e6) <= 650000 and round(run*1e6) >= 1000000),
                heading_good=bool(good[i]), sample_age_s=round(age, 6),
                continuous_valid_s=round(run, 6), counter=int(counter[i]))


def case(label, finish):
    folder = ROOT/"flights"/label
    require(regular(folder/"commit.txt").read_text().strip() == COMMIT, label+" commit differs")
    start = read(folder/"start.json")
    stop = read(folder/"stop-before.json")
    owner = start["owner"]
    require(owner.get("marker")=="launch_project_rgbd.sh" and stop["stopped"] is True and start["reused"] is False
            and start["state"]["connected"] and start["state"]["landed"] and not start["state"]["armed"],
            label+" cold start or disarmed ground state missing")
    supervisor = Path(owner["log"])
    require(supervisor.parent.parent == ROOT/"install04/logs" and supervisor.name == "supervisor.log"
            and re.fullmatch(r"sim-\d{8}-\d{6}", supervisor.parent.name), label+" supervisor path mismatch")
    require(int(regular(supervisor.with_name("supervisor.pid")).read_text()) == int(owner["pid"])
            and str(owner["start_ticks"]).isdigit(), label+" supervisor identity inconsistent")
    pids = re.findall(r"^PX4=(\d+) GAZEBO=(\d+) MAVROS=(\d+) GUI=\d*\s*$",
                      regular(supervisor).read_text(), re.M)
    require(len(pids)==1, label+" missing unique PX4 supervisor process")
    px4log = regular(supervisor.with_name("px4.log"))
    lines = px4log.read_text(errors="replace").splitlines()
    filenames = [line.split("Opened full log file:",1)[1].strip()
                 for line in lines if "Opened full log file:" in line]
    require(len(filenames)==1 and LOG_NAME.fullmatch(filenames[0]), label+" ULog path not unambiguous")
    require(any("closed logfile" in line for line in lines), label+" PX4 logger has not closed ULog")
    path = regular(ROOTFS/filenames[0])
    with path.open('rb') as f:
        require(f.read(7)==b'ULog\x01\x12\x35', label+" invalid ULog header")
    mapping = dict(case=label, original_pass=finish["passed"], start=finish["start"], end=finish["end"],
                   errors=finish.get("errors", []), supervisor=str(supervisor), supervisor_sha256=sha(supervisor),
                   supervisor_pid=owner["pid"], supervisor_start_ticks=owner["start_ticks"], px4_pid=int(pids[0][0]),
                   px4_log_sha256=sha(px4log), start_json_sha256=sha(folder/"start.json"),
                   ulg=str(path), ulg_sha256=sha(path), ulg_bytes=path.stat().st_size)
    postfile=folder/"postflight-state.json"
    if postfile.is_file():
        state=read(postfile).get("state") or {}
        mapping["postflight_state"]={field:state.get(field) for field in ("connected","armed","landed","mode")}
    # A missing optional measurement must not hide a successfully mapped original flight log.
    try:
        mapping.update(analyze_heading(folder, path))
    except (ValueError, KeyError, IndexError, OSError, StopIteration) as exc:
        mapping.update(analysis_error=type(exc).__name__+": "+str(exc), gate_verified_all_requests=False,
                       gate_verified_all_recorded_frames=False)
    return mapping


def analyze_heading(folder, path):
    from pyulog import ULog

    u = ULog(str(path), message_name_filter_list=["vehicle_local_position", "vehicle_status",
                                                   "estimator_status_flags", "vehicle_attitude"])
    p = u.get_dataset("vehicle_local_position").data
    publish = [int(x)/1e6 for x in p["timestamp"]]
    sample = [int(x)/1e6 for x in p["timestamp_sample"]]
    good = [bool(x) for x in p["heading_good_for_control"]]
    counter = p["heading_reset_counter"]
    since, resets = gate_stream(publish, sample, good, counter, p["delta_heading"])
    attitude = u.get_dataset("vehicle_attitude").data
    aq = attitude["quat_reset_counter"]
    quat_resets = [(int(t)/1e6, int(rc)) for t, prev, rc in zip(attitude["timestamp"][1:], aq, aq[1:]) if prev!=rc]
    for r in resets:
        idx=bisect_right(publish,r["stamp"])-1
        r["before_heading_ned_deg"]=finite(math.degrees(p["heading"][idx-1]))
        r["after_heading_ned_deg"]=finite(math.degrees(p["heading"][idx]))
        r["estimated_hagl_m"]=finite(p["dist_bottom"][idx])
        r["matching_quaternion_reset_s"] = next((ts for ts, rc in quat_resets
             if rc==r["counter"] and abs(ts-r["stamp"])<=.04),None)
    status = u.get_dataset("vehicle_status").data
    flags = u.get_dataset("estimator_status_flags").data
    in_air_s = next((int(t)/1e6 for t,s in zip(flags["timestamp"],flags["cs_in_air"]) if s and t>3e6),None)
    trace = read(folder/"result/trace.json")
    states, requests = wait_times(trace)
    for r in requests:
        r["request_gate"] = gate_at(r["request_s"], publish, sample, good, counter, since)
        r["frame_gate"] = gate_at(r["frame_s"], publish, sample, good, counter, since)
    for r in resets:
        last = next((s for s in reversed(states) if s["observed_sim"] <= r["stamp"]),{})
        r["phase"] = last.get("phase")
        r["offset_deg"] = last.get("scan_offset_deg")
    first = min((r["request_s"] for r in requests),default=None)
    after = [r for r in resets if first is not None and r["stamp"] > first]
    acceptance = read(folder/"acceptance.json")
    checks = {r["name"]: r for r in acceptance["checks"]}
    confirmed_frames = [r for r in requests if r["frame_gate"] is not None]
    return dict(trace_sha256=sha(folder/"result/trace.json"),
                initial_mag_mode=int(u.initial_parameters["EKF2_MAG_TYPE"]), heading_good_first_s=next(
                (t for t,v in zip(publish,good) if v),None), in_air_s=in_air_s,
                fcu_armed_ulog=bool(any(v==2 for v in status["arming_state"])),
                final_fcu_arming_state=int(status["arming_state"][-1]),
                first_request_s=first, requests=requests, resets=resets, resets_after_first_request=after,
                gate_verified_all_requests=bool(requests and not after and all(r["request_gate"]
                          and r["request_gate"]["pass_gate"] for r in requests)),
                gate_verified_all_recorded_frames=bool(confirmed_frames and all(
                    r["frame_gate"]["pass_gate"] for r in confirmed_frames)),
                frames_with_timestamp=len(confirmed_frames),
                moving_rgbd_map_p95=checks.get("moving_rgbd",{}).get("map_p95_m"),
                terminal_surface_error=checks.get("final_observation_matches_true_target",{}).get("final_surface_error_m"),
                landed_disarmed=checks.get("landed_disarmed",{}).get("passed"),
                hold={k:checks.get("ten_second_hold_gazebo_truth",{}).get(k) for k in
                      ("passed","samples","covered_core_seconds","world_drift_m","maximum_standoff_error_m")})


def ledger():
    source = regular(ROOT/"V04-attempts.jsonl")
    all_events = [json.loads(line) for line in source.read_text().splitlines() if line.strip()]
    ends = {}
    for event in ("start","finish"):
        records = [r for r in all_events if r.get("event")==event]
        require(Counter(r.get("label") for r in records)==Counter(EXPECTED),
                "Wait for all 16 unique V04 starts and finishes before reading ULogs")
        for row in records:
            if event=="finish":
                require(type(row["passed"]) is bool and row["end"]>=row["start"],"invalid finish metadata")
                ends[row["label"]]=row
    return source, ends


if __name__ == "__main__":
    source, ends = ledger()
    rows = [case(label,ends[label]) for label in EXPECTED]
    for key in ("case","supervisor","ulg","ulg_sha256"):
        require(len({r[key] for r in rows})==len(EXPECTED),"Shared cold-start evidence: "+key)
    require(len({(r["supervisor_pid"],r["supervisor_start_ticks"]) for r in rows})==len(EXPECTED),
            "Repeated supervisor identity")
    json.dump(dict(commit=COMMIT, ledger_sha256=sha(source), cases=rows), sys.stdout,
              indent=2, allow_nan=False)
    print()
