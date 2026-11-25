"""Run the numbered visual SITL acceptance attempts with independent truth checks."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from crosscheck_visual_flight import audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', required=True)
    args = parser.parse_args()
    root = Path(os.environ['UAV_ACCEPTANCE_DIR'])
    ledger = root / (args.batch + '-attempts.jsonl')
    driver = Path(__file__).with_name('run_acceptance_case.sh')
    cases = [('front', 0, 0), ('left45', 54, 45), ('right45', -54, -45),
             ('left90', 103, 90), ('right90', -103, -90)]
    # Reserve this batch ID once. A retry gets its own batch and evidence.
    with ledger.open('x', buffering=1) as log:
        for distance, height, directions in [(2.4, 1.6, cases), (1.5, 1.3, cases[:3])]:
            for direction, bearing, expected in directions:
                for repeat in (1, 2):
                    label = f'{args.batch}-{distance}-{direction}-{repeat}'
                    case = root / 'flights' / label
                    row = dict(label=label, distance=distance, height=height, bearing=bearing,
                               expected=expected, repeat=repeat, start=time.time())
                    print(json.dumps(dict(row, event='start')), file=log)
                    with (root / (label+'.log')).open('x') as console:
                        run = subprocess.run(['bash',str(driver),label,'live',str(distance),
                                              str(height),str(bearing),str(expected)],
                                             stdout=console,stderr=subprocess.STDOUT)
                    if not case.is_dir():
                        raise RuntimeError(f'No attempt directory for {label}, see console')
                    report = audit(case,distance,expected)
                    row.update(event='finish', end=time.time(), exit_code=run.returncode,
                               passed=run.returncode==0 and report['passed'], errors=report['errors'])
                    print(json.dumps(row),file=log)
                    print(json.dumps(row),flush=True)
                    result = case/'result'/'results.json'
                    if not result.exists() or json.loads(result.read_text()).get('final_armed') is not False:
                        raise RuntimeError('Check simulation state before next attempt: '+label)


if __name__ == '__main__':
    main()
