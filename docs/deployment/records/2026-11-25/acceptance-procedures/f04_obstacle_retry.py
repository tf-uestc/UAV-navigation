"""Recheck the original dynamic obstacle setup, retaining every failed attempt."""

import argparse
import json
import os
import subprocess

from f04_long_retry import build, deploy, entry, grade, grade_case, ground, inspect, contact_report, record, repo, root, write_new


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('number', type=int, choices=(2, 3))
    args = parser.parse_args()
    commit = subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'], text=True).strip()
    if commit != '6b58825f8e715d6ac902303b812976ed20693e6e':
        raise ValueError('Restore the same frozen version')
    original = json.loads((root/'F04-1-dynamic-audit.json').read_text())
    if original['failed'] != ['native_delivery']:
        raise ValueError('This retry addresses the missing collision-state witness only')
    ledger = root/'F04-supplement-attempts.jsonl'
    prev = [json.loads(s) for s in ledger.read_text().splitlines()]
    label = f'F04-{args.number}-dynamic'
    case = root/'flights'/label
    if not prev or prev[-1]['event'] != 'finish' or any(x['label'] == label for x in prev) or case.exists():
        raise ValueError('The past trial must finish and the new identifier must be unique')
    env = dict(os.environ, UAV_RUNTIME_CONFIG=str(build/'runtime.env'))
    if not ground(entry, env)['passed']:
        raise ValueError('Live ground state has not been established')
    helpers = dict(physics=contact_report, shutdown=inspect, planning=grade)
    with ledger.open('a', buffering=1) as stream:
        record(stream, event='start', label=label, scenario='dynamic', repeats='F04-1-dynamic', commit=commit)
        with (root/(label+'.log')).open('x') as console:
            p = subprocess.run(['bash', str(deploy/'run_acceptance_case.sh'),label,'dynamic','2.4','1.6','0','0'],
                               env=env, stdout=console, stderr=subprocess.STDOUT)
        safety=ground(entry, env)
        write_new(root/(label+'-ground.json'), safety)
        audit=grade_case(case,'dynamic',deploy,helpers,commit)
        write_new(root/(label+'-audit.json'),audit)
        passed=p.returncode==0 and safety['passed'] and audit['passed']
        record(stream,event='finish',label=label,scenario='dynamic',exit_code=p.returncode,
               ground_verified=safety['passed'],passed=passed,audit=audit)
        return 0 if passed else 1


if __name__=='__main__':
    raise SystemExit(main())
