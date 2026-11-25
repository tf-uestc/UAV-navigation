"""Run and audit the numbered F04 planning trials on a frozen, simulated stack."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
ROUTES = ("straight", "static", "long", "outside")
UNKNOWN_CHECKS = {
    "unknown_never_tracks_and_exits_after_one_observation", "known_free",
    "occupied_goal_rejected", "stale_map_cannot_authorize_translation",
    "restored_input_does_not_revive_old_mission_or_replayed_spline", "fresh_after_recovery",
    "new_explicit_mission_can_execute_after_fresh_fusion", "landed_disarmed",
}


def cases(batch):
    for number in range(1, 6):
        for name in ROUTES:
            yield f"{batch}-{number}-{name}", name
    for name in ("dynamic", "blocked", "unknown"):
        yield f"{batch}-1-{name}", name


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_new(path, value):
    with path.open("x", encoding="utf-8") as f:
        json.dump(value, f, ensure_ascii=False, indent=2)
        f.write("\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record(log, **fields):
    row = dict(utc_epoch_s=time.time(), **fields)
    print(json.dumps(row, ensure_ascii=False), file=log, flush=True)
    os.fsync(log.fileno())
    print(json.dumps(row, ensure_ascii=False), flush=True)


def ground(entry, env):
    samples = []
    for _ in range(3):
        try:
            run = subprocess.run(["bash", str(entry), "status"], capture_output=True, text=True,
                                 timeout=20, env=env)
            result = json.loads(run.stdout)
            state = result.get("state") or {}
            safe = (run.returncode == 0 and state.get("connected") is True
                    and state.get("landed") is True and state.get("armed") is False)
            samples.append(dict(utc_epoch_s=time.time(), safe=safe, state=state,
                                exit_code=run.returncode, error=result.get("error")))
        except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
            samples.append(dict(utc_epoch_s=time.time(), safe=False, error=str(exc)))
        if len(samples) >= 2 and all(s["safe"] for s in samples[-2:]):
            return dict(passed=True, samples=samples)
        time.sleep(.5)
    return dict(passed=False, samples=samples)


def unknown_report(primary, trace):
    checks = {name: name in {r["name"] for r in primary["checks"]} for name in sorted(UNKNOWN_CHECKS)}
    states = [(t, d) for t, kind, d in trace if kind == "mission" and d.get("mission_id")]
    for prefix, terminal in (("outside_view", "FAILED"), ("known_free", "DONE"),
                             ("occupied", "FAILED"), ("stale_map", "FAILED"),
                             ("fresh_after_recovery", "DONE")):
        events = [d for _, d in states if d["mission_id"].startswith(prefix + "-")]
        checks[prefix + "/terminal"] = bool(events) and events[-1]["phase"] == terminal
        if prefix in ("outside_view", "occupied", "stale_map"):
            checks[prefix + "/no_track"] = bool(events) and all(d["phase"] != "TRACK" for d in events)
        if prefix == "outside_view":
            checks["outside_view/reason"] = bool(events) and events[-1]["reason"] == "unknown_trajectory"
    fcu = [d for _, k, d in trace if k == "fcu"]
    checks["armed_then_disarmed"] = any(d.get("armed") is True for d in fcu) and fcu[-1]["armed"] is False
    return dict(passed=all(checks.values()), scenario="unknown", checks=checks)


def delivery_report(case, scenario, primary, trace, deployment, base_grade):
    script = deployment / "crosscheck_trajectory_delivery.py"
    run = subprocess.run([sys.executable, str(script), str(case)], capture_output=True, text=True)
    (case / "native-delivery.stdout").write_text(run.stdout)
    (case / "native-delivery.stderr").write_text(run.stderr)
    strict = json.loads(run.stdout)
    write_new(case / "native-delivery-original.json", strict)
    dispositions = strict["dispositions"]
    accounted = bool(dispositions) and all(d["accounted"] for d in dispositions)
    emergency = bool(strict["native_emergency_recorded"])
    warned = bool(strict["collision_warning_missions"])
    actual_emergency = any(d["native"]["emergency"] for d in dispositions)
    mode = next((c.get("mode") for c in primary["checks"] if c["name"] == "dynamic_replan_executed"), None)
    insert = next(((t, d) for t, k, d in trace if k == "obstacle_inserted"), None)
    followed = set()
    replanned = set()
    if insert:
        t0, obstacle = insert
        for t, k, d in trace:
            if t < t0:
                continue
            if k == "ego_status" and d["status"] == "REPLANNED" and d["reason"] == "collision":
                replanned.add((d["mission_id"], d["goal_id"]))
            if k == "flight" and d["mode"] == "TRACK" and d.get("context") and d.get("trajectory_id") is not None:
                if d["trajectory_id"] > obstacle["first_trajectory_id"]:
                    followed.add((d["context"][0], d["context"][1], d["trajectory_id"]))
    accepted = {(d["native"]["mission_id"], d["native"]["goal_id"], d["native"]["traj_id"])
                for d in dispositions if d["gateway_forwarded"] and (d["executor_ack"] or {}).get("accepted") is True}
    alternatives = sorted(identity for identity in followed & accepted if identity[:2] in replanned)
    continuous = (scenario == "dynamic" and mode == "native_continuous" and bool(alternatives)
                  and base_grade["passed"] and primary["passed"])
    if strict["passed"] and run.returncode == 0:
        outcome = "emergency_packet_observed_and_disposed"
        passed = True
    elif accounted and not warned and not actual_emergency and continuous:
        outcome = "continuous_replan_tracked_without_emergency"
        passed = True
    else:
        outcome = "delivery_evidence_incomplete"
        passed = False
    return dict(passed=passed, outcome=outcome, original_exit=run.returncode, original_pass=strict["passed"],
                accounted_all_actual_packets=accounted, emergency_collision_status_observed=warned,
                native_emergency_observed=actual_emergency,
                emergency_delivery_exercised=emergency, primary_mode=mode,
                post_insertion_replans_tracked=alternatives)


def grade_case(case, scenario, deploy, tools, commit):
    primary = load(case / "result/results.json")
    trace = load(case / "result/trace.json")
    try:
        if scenario == "unknown":
            initial = unknown_report(primary, trace)
        else:
            initial = tools["planning"](case, scenario)
    except (OSError, ValueError, TypeError, KeyError, IndexError, StopIteration) as exc:
        initial = dict(passed=False, checks=dict(planning_evidence_available=False), error=repr(exc))
    write_new(case / "independent-planning.json", initial)
    try:
        physical = tools["physics"](case)
    except (OSError, ValueError, TypeError, KeyError, IndexError, StopIteration) as exc:
        physical = dict(passed=False, checks=dict(raw_physics_available=False), error=repr(exc))
    report = dict(scenario=scenario, checks=dict(initial["checks"]), physics_coverage=physical)
    report["checks"]["contact_stream_observed"] = physical["passed"]
    report["passed"] = all(report["checks"].values())
    write_new(case / "physics-original-audit.json", report)
    physics_ok = physical["passed"]
    if (scenario != "unknown" and not physics_ok and primary.get("passed") is True
            and list(k for k, v in physical["checks"].items() if not v) == ["brackets_entire_flight"]):
        try:
            refined = tools["shutdown"](case, report, deploy)
        except (OSError, ValueError, TypeError, KeyError, IndexError, StopIteration) as exc:
            refined = dict(passed=False, error=repr(exc))
        write_new(case / "physics-grounded-shutdown.json", refined)
        physics_ok = refined["passed"]
    checks = dict(report["checks"])
    checks["contact_stream_observed"] = physics_ok
    checks["frozen_runtime"] = ((case / "commit.txt").read_text().strip() == commit
                                and not (case / "source-status.txt").read_text().strip())
    checks["runner_primary"] = primary.get("passed") is True and primary.get("final_armed") is False
    if scenario in ("dynamic", "blocked"):
        # Physical evidence is applied separately; delivery can still be classified when a
        # conservatively bracketed recorder endpoint is resolved by the supplementary proof.
        semantic = dict(initial)
        semantic["passed"] = all(v for k, v in initial["checks"].items() if k != "contact_stream_observed")
        packets = delivery_report(case, scenario, primary, trace, deploy, semantic)
        write_new(case / "native-delivery-interpretation.json", packets)
        checks["native_delivery"] = packets["passed"]
    return dict(passed=all(checks.values()), checks=checks, failed=[k for k, ok in checks.items() if not ok])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", default="F04")
    parser.add_argument("--commit", help="full 40-character business Git commit for this installation")
    parser.add_argument("--root", default=os.environ.get("UAV_ACCEPTANCE_DIR"), type=Path)
    parser.add_argument("--build", default=os.environ.get("UAV_ACCEPTANCE_BUILD"), type=Path)
    parser.add_argument("--audit-dir", type=Path, default=HERE)
    parser.add_argument("--plan", action="store_true", help="list the flight IDs without contacting any simulator")
    parser.add_argument("--resume", action="store_true", help="continue after fully recorded attempts")
    args = parser.parse_args()
    if not re.fullmatch(r"F04(?:-[A-Za-z0-9]+)?", args.batch):
        parser.error("Use an F04 batch ID, for example F04 or F04-R1")
    schedule = list(cases(args.batch))
    if args.plan:
        print(json.dumps(schedule, indent=2))
        return 0
    import fcntl
    if not args.commit or not re.fullmatch(r"[a-f0-9]{40}", args.commit) or not args.root or not args.build:
        parser.error("Supply --commit plus both the acceptance root and the isolated build")
    root, build, audit_dir = args.root.resolve(), args.build.resolve(), args.audit_dir.resolve()
    repo = build / "UAV-navigation"
    deploy = repo / "tools/deployment"
    commit = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    if commit != args.commit or subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain"]):
        raise RuntimeError("Frozen clean source and supplied commit must agree")
    driver = deploy / "run_acceptance_case.sh"
    entry = deploy / "uav_runtime.sh"
    observer = HERE / "f04_unknown_observer.py"
    sys.path[:0] = [str(deploy), str(audit_dir)]
    from verify_planning import contact_report
    from verify_grounded_shutdown import inspect
    from crosscheck_planning import grade
    tools = dict(physics=contact_report, shutdown=inspect, planning=grade)
    scripts = [Path(__file__).resolve(), observer, driver, entry,
               Path(sys.modules["verify_planning"].__file__), Path(sys.modules["verify_grounded_shutdown"].__file__),
               Path(sys.modules["crosscheck_planning"].__file__), deploy / "crosscheck_trajectory_delivery.py",
               deploy / "validate_planning_sitl.py", deploy / "validate_unknown_sitl.py",
               deploy / "record_physics_contacts.py", deploy / "observe_physics_contacts.py"]
    fingerprints = {str(s): sha(s) for s in scripts}
    marker = "/usr/bin/python3 -u tools/deployment/validate_unknown_sitl.py"
    original = driver.read_text()
    if original.count(marker) != 1:
        raise RuntimeError("Unexpected unknown-space test entry in the selected runner")
    env = dict(os.environ, UAV_ACCEPTANCE_DIR=str(root), UAV_ACCEPTANCE_BUILD=str(build),
               UAV_RUNTIME_CONFIG=str(build / "runtime.env"))
    lock_file = root / ".f04-planning.lock"
    ledger = root / (args.batch + "-attempts.jsonl")
    with lock_file.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.resume:
            previous = [json.loads(line) for line in ledger.read_text().splitlines()]
            if not previous or previous[0].get("event") != "batch" or previous[0]["fingerprints"] != fingerprints or previous[0]["commit"] != commit:
                raise RuntimeError("Cannot resume with different source or instrumentation")
            started = [x["label"] for x in previous if x["event"] == "start"]
            finished = [x["label"] for x in previous if x["event"] == "finish"]
            if started != finished or len(set(started)) != len(started) or started != [label for label, _ in schedule[:len(started)]]:
                raise RuntimeError("Incomplete attempt or invalid prefix; preserve evidence and inspect before continuing")
            failed = sum(not x["passed"] for x in previous if x["event"] == "finish")
            if len(started) == len(schedule) and any(x["event"] == "complete" for x in previous):
                return int(failed != 0)
            safety = ground(entry, env) if started else {"passed": True, "samples": []}
            if not safety["passed"]:
                raise RuntimeError("Cannot verify landed and disarmed before resume")
        else:
            started, failed = [], 0
        adjusted = root / (args.batch + "-unknown-instrumented.sh")
        with ledger.open("a" if args.resume else "x", buffering=1) as log:
            if not args.resume:
                unknown_entry = "/usr/bin/python3 -u " + shlex.quote(str(observer)) + " tools/deployment/validate_unknown_sitl.py"
                with adjusted.open("x") as f:
                    f.write(original.replace(marker, unknown_entry))
                record(log, event="batch", batch=args.batch, commit=commit, schedule=schedule,
                       fingerprints=fingerprints, unknown_runner_sha256=sha(adjusted))
            elif sha(adjusted) != previous[0]["unknown_runner_sha256"]:
                raise RuntimeError("Unknown-space instrumentation changed since the first attempt")
            for label, scenario in schedule[len(started):]:
                case = root / "flights" / label
                console_name = root / (label + ".log")
                if case.exists() or console_name.exists():
                    raise RuntimeError("Trial ID already exists: " + label)
                with console_name.open("x") as console:
                    record(log, event="start", label=label, scenario=scenario)
                    runner = adjusted if scenario == "unknown" else driver
                    command = ["bash", str(runner), label, scenario, "2.4", "1.6", "0", "0"]
                    try:
                        status = subprocess.call(command, env=env, stdout=console, stderr=subprocess.STDOUT)
                    except OSError as exc:
                        print(str(exc), file=console)
                        status = None
                # A failed validator still reaches this separate live FCU check. Never restart a
                # simulation which could still be armed just because a prior JSON said disarmed.
                safe = ground(entry, env)
                write_new(root / (label + "-ground.json"), safe)
                try:
                    audit = grade_case(case, scenario, deploy, tools, commit)
                except (OSError, ValueError, TypeError, KeyError, StopIteration, IndexError, RuntimeError) as exc:
                    audit = dict(passed=False, error=repr(exc))
                passed = status == 0 and safe["passed"] and audit["passed"]
                failed += int(not passed)
                write_new(root / (label + "-audit.json"), audit)
                record(log, event="finish", label=label, scenario=scenario, exit_code=status,
                       ground_verified=safe["passed"], passed=passed, audit=audit)
                if not safe["passed"]:
                    return 2
            record(log, event="complete", started=len(schedule), passed=len(schedule)-failed, failed=failed)
    return int(failed != 0)


if __name__ == "__main__":
    sys.exit(main())
