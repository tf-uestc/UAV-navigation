#!/usr/bin/env python3
"""Export deterministic semantic replay traces; this does not simulate physics."""
import argparse
import json
import sys
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/"tests/offline"))
from test_offline_pipeline import PipelineReplayTests

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    traces={}
    cases=[name for name in unittest.defaultTestLoader.getTestCaseNames(PipelineReplayTests)
           if name!="test_latency_cases_keep_correct_request"]
    for name in cases:
        case=PipelineReplayTests(name)
        result=unittest.TestResult()
        case.run(result)
        if not result.wasSuccessful():
            raise RuntimeError("replay failed: "+name)
        traces[name]=case.h.trace
    target=args.output if args.output.is_absolute() else ROOT/args.output
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open("w",encoding="utf-8",newline="\n") as handle:
        json.dump(dict(scope="offline semantic events with ideal feedback; NOT physical flight",
                       traces=traces),handle,ensure_ascii=False,indent=2,allow_nan=False)
    print("Exported {} scenario traces".format(len(traces)))
    return 0
if __name__=="__main__":
    sys.exit(main())
