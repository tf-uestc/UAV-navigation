"""Serial physical SITL recheck of landing confirmation. No visual API is involved."""
import json
import os
from pathlib import Path
import subprocess
import sys

from f04_matrix import ground, record, write_new
from f04_targeted_checks import ensure_cloud_executor_exited, substitute_once


def main():
    root=Path(os.environ['UAV_ACCEPTANCE_DIR']); build=Path(os.environ['UAV_ACCEPTANCE_BUILD'])
    assert build==root/'install05-landing'
    source=build/'UAV-navigation'; tools=source/'tools/deployment'; entry=tools/'uav_runtime.sh'
    owner=root/'install04'/'UAV-navigation/tools/deployment/uav_runtime.sh'
    commit=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    if commit!=sys.argv[1] or subprocess.check_output(['git','-C',str(source),'status','--porcelain']):
        raise ValueError('A clean, identified candidate is required')
    for names,expected in [('V04',16),('S04',2)]:
        rows=[json.loads(s) for s in (root/(names+'-attempts.jsonl')).read_text().splitlines()]
        a=[r['label'] for r in rows if r['event']=='start']; b=[r['label'] for r in rows if r['event']=='finish']
        if a!=b or len(set(b))!=expected or len(b)!=expected: raise ValueError('Unfinished prior batch')
    ensure_cloud_executor_exited()
    prev=dict(os.environ,UAV_RUNTIME_CONFIG=str(root/'install04/runtime.env'))
    if not ground(owner,prev)['passed']: raise ValueError('Cannot confirm previous simulation ground state')
    subprocess.run(['bash',str(owner),'stop'],env=prev,check=True)
    for proc in ('px4','gzserver','gzclient'):
        if subprocess.run(['pgrep','-x',proc],stdout=subprocess.DEVNULL).returncode!=1:
            raise ValueError('Previous simulator still running: '+proc)
    out=root/'isolated'/'t05-landing'; out.mkdir()
    base=Path(__file__).resolve().parent
    observer=str(base/'observe_landing.py'); contacts=str(base/'f04_unknown_observer.py')
    driver=out/'run.sh'; command=(tools/'run_acceptance_case.sh').read_text()
    command=substitute_once(command,'/usr/bin/python3 -u tools/deployment/validate_planning_sitl.py "$scenario"',
        '/usr/bin/python3 -u '+observer+' "$r/landing-handshake.json" /usr/bin/python3 -u tools/deployment/validate_planning_sitl.py "$scenario"')
    command=substitute_once(command,'/usr/bin/python3 -u tools/deployment/validate_unknown_sitl.py --out',
        '/usr/bin/python3 -u '+observer+' "$r/landing-handshake.json" /usr/bin/python3 -u '+contacts+' tools/deployment/validate_unknown_sitl.py --out')
    driver.write_text(command)
    env=dict(os.environ,UAV_RUNTIME_CONFIG=str(build/'runtime.env'))
    with (root/'T05-attempts.jsonl').open('x',buffering=1) as ledger:
        for idx,scenario in enumerate(('unknown','straight','static','dynamic'),1):
            label=f'T05-{idx}-{scenario}'; case=root/'flights'/label
            if case.exists() or (idx>1 and not ground(entry,env)['passed']): raise ValueError('Start guard failed')
            record(ledger,event='start',label=label,scenario=scenario,commit=commit)
            with (root/(label+'.log')).open('x') as console:
                run=subprocess.run(['bash',str(driver),label,scenario,'2.4','1.6','0','0'],env=env,stdout=console,stderr=subprocess.STDOUT)
            safe=ground(entry,env); write_new(root/(label+'-ground.json'),safe)
            evidence=case/'landing-handshake.json'; info=json.loads(evidence.read_text()) if evidence.exists() else {'passed':False,'error':'missing handshake'}
            primary=case/'result/results.json'; business=json.loads(primary.read_text()) if primary.exists() else {}
            physics=case/'result/physics-contacts.json'; contact=json.loads(physics.read_text()) if physics.exists() else {}
            detail={k:v for k,v in info.items() if k!='events'}
            result=dict(primary=business.get('passed'),contact_stream=contact.get('passed'),ground=safe['passed'],handshake=detail)
            write_new(root/(label+'-audit.json'),result)
            record(ledger,event='finish',label=label,exit_code=run.returncode,ground_verified=safe['passed'],
                   passed=run.returncode==0 and safe['passed'] and business.get('passed') is True and contact.get('passed') is True and info['passed'],audit=result)
            if not safe['passed']: raise RuntimeError('Ground state unavailable after '+label)


if __name__=='__main__':
    main()
