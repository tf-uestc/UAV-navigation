"""Repeat the diagnosed long-route interruption once after the planned F04 matrix."""
import json
import os
from pathlib import Path
import subprocess
import sys

from f04_matrix import grade_case, ground, record, write_new


root = Path(os.environ["UAV_ACCEPTANCE_DIR"])
build = Path(os.environ["UAV_ACCEPTANCE_BUILD"])
repo = build / "UAV-navigation"
deploy = repo / "tools/deployment"
entry = deploy / "uav_runtime.sh"
sys.path.insert(0, str(deploy))
from crosscheck_planning import grade
from verify_planning import contact_report
from verify_grounded_shutdown import inspect


def main():
    commit = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    if commit != "6b58825f8e715d6ac902303b812976ed20693e6e":
        raise ValueError("Retry requires the same frozen version")
    prior = [json.loads(line) for line in (root / "F04-attempts.jsonl").read_text().splitlines()]
    if prior[-1]["event"] != "complete" or prior[-1]["started"] != 23:
        raise ValueError("Planned matrix has not finished")
    failed = [r["label"] for r in prior if r["event"] == "finish" and not r["passed"]]
    if "F04-2-long" not in failed:
        raise ValueError("Expected registered F04-2-long failure")
    label = "F04-6-long"
    case = root / "flights" / label
    env = dict(os.environ, UAV_RUNTIME_CONFIG=str(build / "runtime.env"))
    if case.exists() or not ground(entry, env)["passed"]:
        raise ValueError("Retry already started or safe ground state unconfirmed")
    tools = dict(physics=contact_report, shutdown=inspect, planning=grade)
    with (root / "F04-supplement-attempts.jsonl").open("x", buffering=1) as ledger:
        record(ledger, event="start", label=label, scenario="long", repeats="F04-2-long", commit=commit)
        with (root / (label + ".log")).open("x") as console:
            trial = subprocess.run(["bash", str(deploy / "run_acceptance_case.sh"), label, "long",
                                    "2.4", "1.6", "0", "0"], stdout=console, stderr=subprocess.STDOUT, env=env)
        safe = ground(entry, env)
        write_new(root / (label + "-ground.json"), safe)
        audit = grade_case(case, "long", deploy, tools, commit)
        write_new(root / (label + "-audit.json"), audit)
        passed = trial.returncode == 0 and safe["passed"] and audit["passed"]
        record(ledger, event="finish", label=label, scenario="long", exit_code=trial.returncode,
               passed=passed, ground_verified=safe["passed"], audit=audit)
        return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
