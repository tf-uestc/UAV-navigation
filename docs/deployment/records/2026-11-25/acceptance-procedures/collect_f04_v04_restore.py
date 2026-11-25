"""Read archived F04/V04 startup evidence and print a credential-free JSON snapshot."""

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys


PARAMETERS = {"height", "visual_takeoff_height", "standoff", "require_sitl_heading"}
NODES = {"ego_planner_node", "flight_executor", "segment_mission", "sitl_heading_readiness",
         "detector_node", "camera_pose_bridge", "managed_planner_node", "mavros"}
PID = re.compile(r"process\[([^\]]+?)-\d+\]: started with pid \[(\d+)\]")
PARAM = re.compile(r"^\s*\* /segment_mission/([a-z_]+):\s*([^\s]+)\s*$")
TERMINAL_COLORS = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
SERVICES = ("PX4", "GAZEBO", "MAVROS")


def expected_labels():
    f04 = ["F04-%d-%s" % (i, n) for i in range(1, 6) for n in ("straight", "static", "long", "outside")]
    f04 += ["F04-1-" + n for n in ("dynamic", "blocked", "unknown")]
    v04 = ["V04-%s-%s-%d" % (d, n, i) for d, ns in [
        ("2.4", ("front", "left45", "right45", "left90", "right90")),
        ("1.5", ("front", "left45", "right45"))] for n in ns for i in (1, 2)]
    return dict(F04=f04, V04=v04)


def inside(path, root):
    try:
        path.resolve().relative_to(root)
        return True
    except (ValueError, OSError, RuntimeError):
        return False


def archived(path, root):
    return path.is_file() and not path.is_symlink() and inside(path, root)


def json_object(path, root):
    if archived(path, root):
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(value, dict):
                return value
        except (OSError, ValueError):
            pass
    return None


def hex_commit(value):
    return value if isinstance(value, str) and re.fullmatch(r"[a-f0-9]{40}", value) else None


def git_commit(repo):
    if not repo.is_dir():
        return None
    try:
        res = subprocess.run(["git", "--no-optional-locks", "-C", str(repo), "rev-parse", "--verify", "HEAD"],
                             capture_output=True, text=True, timeout=8)
        if res.returncode == 0:
            return hex_commit(res.stdout.strip())
    except (OSError, subprocess.TimeoutExpired):
        pass
    return None


def file_record(path, root):
    ok = archived(path, root)
    return dict(path=str(path), present=ok,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest() if ok else None)


def finite_or_bool(name, raw):
    if name == "require_sitl_heading":
        return {"true": True, "false": False, "1": True, "0": False}.get(raw.lower())
    try:
        value = float(raw)
        return value if math.isfinite(value) else None
    except ValueError:
        return None


def roslaunch_fields(path, root):
    nodes = defaultdict(list)
    params = {}
    if archived(path, root):
        with path.open(encoding="utf-8", errors="replace") as stream:
            for line in stream:
                line = TERMINAL_COLORS.sub("", line)
                node = PID.search(line)
                if node and node[1].lstrip("/") in NODES:
                    nodes[node[1].lstrip("/")].append(int(node[2]))
                param = PARAM.match(line)
                if param and param[1] in PARAMETERS:
                    params[param[1]] = finite_or_bool(param[1], param[2])
    return dict(nodes), params


def supervisor_evidence(owner, build):
    # The only accepted log pointer is the known supervisor leaf of a cold-start directory.
    # Resolve before reading, so an arbitrary remote log pointer is never followed.
    ret = dict(supervisor_log=None, supervisor_pid_file=None, supervisor_pid_matches=None,
               launch_pids={}, mavros_node_pids=[])
    candidate = owner.get("log")
    if not isinstance(candidate, str):
        return ret
    path = Path(candidate)
    if (path.name != "supervisor.log" or not re.fullmatch(r"sim-\d{8}-\d{6}", path.parent.name)
            or path.parent.parent != build / "logs" or not archived(path, build)):
        return ret
    ret["supervisor_log"] = str(path)
    pidfile = path.with_name("supervisor.pid")
    if archived(pidfile, build):
        value = pidfile.read_text().strip()
        if value.isdigit():
            ret["supervisor_pid_file"] = int(value)
            ret["supervisor_pid_matches"] = int(value) == owner.get("pid")
    with path.open(encoding="utf-8", errors="replace") as log:
        for line in log:
            match = re.fullmatch(r"PX4=(\d+) GAZEBO=(\d+) MAVROS=(\d+) GUI=(?:\d*)\s*", line)
            if match:
                ret["launch_pids"] = dict(zip(SERVICES, map(int, match.groups())))
    nodes, _ = roslaunch_fields(path.with_name("mavros.log"), build)
    ret["mavros_node_pids"] = nodes.get("mavros", [])
    return ret


def ledger_events(root, batch, schedule):
    path = root / (batch + "-attempts.jsonl")
    events = defaultdict(Counter)
    invalid = 0
    if archived(path, root):
        with path.open(encoding="utf-8", errors="replace") as f:
            for line in f:
                try:
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        raise ValueError()
                except ValueError:
                    invalid += 1
                    continue
                if row.get("event") in ("start", "finish"):
                    if row.get("label") in schedule:
                        events[row["label"]][row["event"]] += 1
                    else:
                        invalid += 1
    return events, invalid


def case_evidence(label, expected, root, build, events):
    folder = root / "flights" / label
    begin = json_object(folder / "start.json", root) or {}
    stop = json_object(folder / "stop-before.json", root) or {}
    owner = begin.get("owner") if isinstance(begin.get("owner"), dict) else {}
    state = begin.get("state") if isinstance(begin.get("state"), dict) else {}
    pids = supervisor_evidence(owner, build)
    status_file = folder / "source-status.txt"
    clean = not status_file.read_bytes().strip() if archived(status_file, root) else None
    commit_file = folder / "commit.txt"
    commit = hex_commit(commit_file.read_text().strip()) if archived(commit_file, root) else None
    nodes, params = roslaunch_fields(folder / "launch.log", root)
    pid = owner.get("pid")
    ticks = str(owner.get("start_ticks", ""))
    ticks = int(ticks) if ticks.isdigit() else None
    ground = all((state.get("connected") is True, state.get("armed") is False, state.get("landed") is True))
    onboard = label.startswith("V04-")
    nominal_distance = float(label.split("-")[1]) if onboard else 2.4
    checks = dict(
        ledger_start_unique=events["start"] == 1, ledger_finish_unique=events["finish"] == 1,
        clean_source_at_start=clean is True, expected_commit_at_start=commit == expected,
        stopped_before=stop.get("stopped") is True, fresh_start=begin.get("reused") is False,
        started_on_ground_disarmed=ground, supervisor_pid_verified=pids["supervisor_pid_matches"] is True,
        supervisor_ticks_saved=ticks is not None, all_supervisor_roles_seen=len(pids["launch_pids"]) == 3,
        business_nodes_seen=all(nodes.get(n) for n in ("ego_planner_node", "flight_executor", "segment_mission")),
        heading_gate_policy_observed=params.get("require_sitl_heading") is onboard,
        heading_node_as_configured=bool(nodes.get("sitl_heading_readiness")) == onboard and
                                  params.get("require_sitl_heading") is onboard,
        mission_height_as_scheduled=params.get("height") == (1.3 if nominal_distance == 1.5 else 1.6),
        takeoff_height_as_scheduled=params.get("visual_takeoff_height") == 1.6,
        standoff_as_scheduled=params.get("standoff") == nominal_distance,
    )
    return dict(label=label, business_commit=commit, clean_source=clean, start_reused=begin.get("reused")
                if isinstance(begin.get("reused"), bool) else None,
                startup_state={k: state.get(k) if isinstance(state.get(k), bool) else None
                               for k in ("connected", "armed", "landed")},
                supervisor_pid=pid if type(pid) is int else None, supervisor_start_ticks=ticks,
                supervisor=pids, roslaunch_node_pids=nodes, launch_parameters=params, checks=checks)


def installation(build):
    repo = build / "UAV-navigation"
    pkg = build / "business-install/share/uav_vln_bringup"
    egopkg = build / "ego-install/share/ego_planner"
    node = build / "business-install/lib/uav_vln_bringup/sitl_heading_readiness.py"
    source = repo / "ws/src/uav_vln_bringup/scripts/sitl_heading_readiness.py"
    frozen = {"runtime_entry": repo / "tools/deployment/uav_runtime.sh",
              "case_runner": repo / "tools/deployment/run_acceptance_case.sh",
              "business_package": pkg / "package.xml", "ego_package": egopkg / "package.xml",
              "installed_launch": pkg / "launch/segment_verified.launch",
              "installed_heading_node": node, "heading_source": source}
    files = {name: file_record(path, build) for name, path in frozen.items()}
    body_match = None
    if files["heading_source"]["present"] and files["installed_heading_node"]["present"]:
        # catkin_install_python may replace the shebang only.
        body_match = node.read_bytes().splitlines()[1:] == source.read_bytes().splitlines()[1:]
    implementation = node.read_text() if files["installed_heading_node"]["present"] else ""
    onboard_source = all(token in implementation for token in (
        "px4-listener", "vehicle_local_position", "heading_good_for_control", "/uav/sitl/heading_ready"))
    runner = frozen["case_runner"].read_text() if files["case_runner"]["present"] else ""
    guard = all(line in runner for line in (
        'test "$(realpath "$(rospack find ego_planner)")" = "$(realpath "$d/ego-install/share/ego_planner")"',
        'test "$(realpath "$(rospack find uav_vln_bringup)")" = "$(realpath "$d/business-install/share/uav_vln_bringup")"',
    ))
    overlay = json_object(build / "source-overlay-sha256.json", build)
    return dict(build=str(build), git_business_commit=git_commit(repo), ego_base=git_commit(build / "ego-planner"),
                source_overlay_entry_count=len(overlay) if overlay is not None else None,
                package_paths=dict(uav_vln_bringup=str(pkg), ego_planner=str(egopkg)),
                runner_checks_both_resolved_package_paths_before_launch=guard,
                installed_files=files, heading_installed_body_matches_source=body_match,
                heading_bridge=dict(node="/sitl_heading_readiness", output_topic="/uav/sitl/heading_ready",
                                    px4_uorb_input="vehicle_local_position.heading_good_for_control",
                                    listener_and_topics_identified_in_installed_code=onboard_source))


def collect(root, build, expected):
    schedule = expected_labels()
    restore = installation(build)
    rows = []
    missing = []
    invalid_ledgers = {}
    for batch, labels in schedule.items():
        events, invalid_ledgers[batch] = ledger_events(root, batch, labels)
        for label in labels:
            if label in events or (root / "flights" / label).is_dir():
                rows.append(case_evidence(label, expected, root, build, events[label]))
            else:
                missing.append(label)
    owners = defaultdict(list)
    logs = defaultdict(list)
    pids = {role: defaultdict(list) for role in SERVICES}
    for r in rows:
        if r["supervisor_pid"] and r["supervisor_start_ticks"]:
            owners[(r["supervisor_pid"], r["supervisor_start_ticks"])].append(r["label"])
        info = r["supervisor"]
        if info["supervisor_log"]:
            logs[info["supervisor_log"]].append(r["label"])
        for role, pid in info["launch_pids"].items():
            pids[role][pid].append(r["label"])
    for r in rows:
        signature = (r["supervisor_pid"], r["supervisor_start_ticks"])
        r["checks"]["unique_supervisor_identity_and_log"] = (len(owners.get(signature, [])) == 1
            and len(logs.get(r["supervisor"]["supervisor_log"], [])) == 1)
        r["complete_startup_evidence"] = all(r["checks"].values())
    counts = {}
    for batch, labels in schedule.items():
        selected = [r for r in rows if r["label"] in labels]
        counts[batch] = dict(planned=len(labels), observed=len(selected),
                             full_startup_evidence=sum(r["complete_startup_evidence"] for r in selected))
    recovered = (restore["git_business_commit"] == expected
                 and restore["ego_base"] == "bfda51284c8c1b476043255a8145ef925a3778a5"
                 and restore["source_overlay_entry_count"] == 0
                 and restore["runner_checks_both_resolved_package_paths_before_launch"]
                 and restore["heading_bridge"]["listener_and_topics_identified_in_installed_code"]
                 and restore["heading_installed_body_matches_source"] is True and
                 all(f["present"] for f in restore["installed_files"].values()))
    complete = recovered and not missing and not any(invalid_ledgers.values()) and all(
        r["complete_startup_evidence"] for r in rows)
    return dict(collected_utc=datetime.now(timezone.utc).isoformat(), expected_commit=expected,
                complete=complete, frozen_installation_verified=recovered, counts=counts,
                missing_labels=missing, invalid_or_unexpected_ledger_records=invalid_ledgers,
                distinct_supervisor_identities=len(owners), distinct_supervisor_logs=len(logs),
                distinct_launch_pids={role: len(group) for role, group in pids.items()},
                reused_numeric_launch_pids={role: {pid: labels for pid, labels in group.items() if len(labels)>1}
                                            for role, group in pids.items()},
                installation=restore, cases=rows,
                scope="Startup and logged ROS launch parameters only. PID in MAVROS supervisor field is the "
                      "roslaunch child; mavros_node_pids comes separately from mavros.log. Numeric PID reuse "
                      "over time is reported separately from supervisor PID/start-tick identity. "
                      "Flight acceptance and measured flight altitude use their independent trial records.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="experiment root containing flights and ledgers")
    parser.add_argument("--build", type=Path, required=True, help="fixed, isolated build/install directory")
    parser.add_argument("--commit", required=True, help="expected full 40-character business commit")
    parser.add_argument("--require-complete", action="store_true", help="exit 1 if any planned startup is unverified")
    args = parser.parse_args()
    if not hex_commit(args.commit):
        parser.error("Use the entire lowercase Git commit")
    if not args.root.is_dir() or not args.build.is_dir():
        parser.error("The experiment and installation directories must exist")
    report = collect(args.root.resolve(), args.build.resolve(), args.commit)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return int(args.require_complete and not report["complete"])


if __name__ == "__main__":
    sys.exit(main())
