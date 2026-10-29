"""Compatibility entry point; the maintained tests are in tests/offline."""
import sys
from pathlib import Path
import runpy

if __name__=="__main__":
    runpy.run_path(str(Path(__file__).resolve().parent/"tools/run_offline_tests.py"),run_name="__main__")
