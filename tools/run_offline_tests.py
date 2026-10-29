#!/usr/bin/env python3
"""Run offline tests from any working directory and optionally save evidence."""
import argparse
import hashlib
import io
import platform
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--log",type=Path)
    parser.add_argument("--pattern",default="test_*.py")
    args=parser.parse_args()
    output=io.StringIO()
    suite=unittest.defaultTestLoader.discover(str(ROOT/"tests/offline"),pattern=args.pattern)
    result=unittest.TextTestRunner(stream=output,verbosity=2).run(suite)
    digest=hashlib.sha256()
    paths=sorted(list((ROOT/"ws/src/uav_vln_bringup/scripts").rglob("*.py"))+
                 list((ROOT/"tests/offline").rglob("*.py")),key=lambda p:p.relative_to(ROOT).as_posix())
    for path in paths:
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    header=("UTC: "+datetime.now(timezone.utc).isoformat()+"\nPython: "+platform.python_version()+
            "\nPlatform: "+platform.platform()+"\nSource/test SHA256: "+digest.hexdigest()+
            "\nScope: offline logic and ROS-shaped stand-ins; ROS/PX4/Gazebo not exercised.\n\n")
    text=header+output.getvalue()
    if args.log:
        log=args.log if args.log.is_absolute() else ROOT/args.log
        log.parent.mkdir(parents=True,exist_ok=True)
        log.write_text(text,encoding="utf-8")
    print("Offline: {} tests, {} failures, {} errors; {}".format(
        result.testsRun,len(result.failures),len(result.errors),"PASS" if result.wasSuccessful() else "FAIL"))
    if not result.wasSuccessful(): print(output.getvalue())
    return 0 if result.wasSuccessful() else 1

if __name__=="__main__":
    sys.exit(main())
