"""Wrap the existing unknown-space SITL test in the Gazebo contact recorder."""

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("validator", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    recorder = args.validator.with_name("record_physics_contacts.py")
    command = [sys.executable, "-u", str(recorder), "--out", str(args.out), "--model", "guard_box"]
    with (args.out / "contact-recorder.log").open("x") as console:
        observer = subprocess.Popen(command, stdout=console, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 24
            while not (args.out / "contacts-ready.json").exists():
                if observer.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Gazebo physical contact recorder did not become ready")
                time.sleep(0.15)
            code = subprocess.call([sys.executable, "-u", str(args.validator), "--out", str(args.out)])
        finally:
            if observer.poll() is None:
                observer.send_signal(signal.SIGTERM)
            try:
                observer.wait(timeout=30)
            except subprocess.TimeoutExpired:
                observer.kill()
                observer.wait()
    coverage = json.loads((args.out / "physics-contacts.json").read_text())
    return code if code else int(observer.returncode != 0 or coverage.get("passed") is not True)


if __name__ == "__main__":
    sys.exit(main())
