"""Two separately numbered Gazebo-only follow-ups to the final planning matrix."""

import hashlib
import json
import os
from pathlib import Path
import subprocess

from f04_long_retry import build, deploy, entry, grade, grade_case, ground, inspect, contact_report, record, repo, root, write_new


def substitute_once(source, old, new):
    if source.count(old) != 1:
        raise RuntimeError('Frozen test fixture differs from the expected input')
    return source.replace(old, new)


def successful_recovery(trace):
    matching = [(t, d) for t, k, d in trace if k == 'mission'
                and (d.get('mission_id') or '').startswith('fresh_after_recovery-')]
    goals = {d['mission_id'] for _, d in matching}
    done = [t for t,d in matching if d['phase'] == 'DONE' and d['reason'] == 'destination_reached']
    landing = [t for t,k,d in trace if k == 'flight_command' and d.get('command') == 'LAND']
    failed = [d.get('reason') for _,d in matching if d['phase'] == 'FAILED']
    done_time = min(done) if done else None
    land_time = min(landing) if landing else None
    post = [(t,d['phase']) for t,d in matching if done_time is not None and t >= done_time]
    terminal_ok = (len(goals) == 1 and not failed and done_time is not None and land_time is not None
                   and done_time < land_time and any(p=='LANDED' for _,p in post)
                   and all(p in ('DONE','LANDING','LANDED') for _,p in post))
    return dict(passed=terminal_ok, mission_ids=sorted(goals), first_done=done_time,
                first_land_command=land_time, phases_after_done=sorted({p for _,p in post}), failed=failed)


def ensure_cloud_executor_exited():
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            argv=(item/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(arg.rsplit(b'/',1)[-1] in {b'run_visual_matrix.py',b'run_acceptance_case.sh',b'validate_search_sitl.py'} for arg in argv if arg):
            raise RuntimeError('Another trial still owns the simulator')


def main():
    commit = subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()
    if commit != '6b58825f8e715d6ac902303b812976ed20693e6e' or subprocess.check_output(['git','-C',str(repo),'status','--porcelain']):
        raise ValueError('Frozen clean source required')
    ledger_rows = [json.loads(s) for s in (root/'V04-attempts.jsonl').read_text().splitlines()]
    starts=[v['label'] for v in ledger_rows if v['event']=='start']
    ends=[v['label'] for v in ledger_rows if v['event']=='finish']
    if len(set(ends)) != 16 or len(ends)!=16 or starts!=ends:
        raise ValueError('The cloud campaign has not finished')
    ensure_cloud_executor_exited()
    env = dict(os.environ, UAV_RUNTIME_CONFIG=str(build/'runtime.env'))
    if not ground(entry,env)['passed']:
        raise RuntimeError('Previous simulation must be grounded and disarmed twice')
    subprocess.run(['bash',str(entry),'stop'],env=env,check=True)
    for program in ('px4','gzserver','gzclient'):
        status=subprocess.run(['pgrep','-x',program],stdout=subprocess.DEVNULL).returncode
        if status!=1:
            raise RuntimeError('Previous simulation has not fully stopped: '+program)
    out = root/'isolated'/'s04-targeted'
    out.mkdir()
    fixture_path = out/'validate_offset_box.py'
    source = (deploy/'validate_planning_sitl.py').read_text()
    fixture = substitute_once(source,'center = p.x + 1.14, p.y\n','center = p.x + 1.14, p.y + 0.30\n')
    fixture = substitute_once(fixture, 'start_time_ns=msg.start_time.to_nsec()))',
        'start_time_ns=msg.start_time.to_nsec(), order=int(msg.order), knots=list(msg.knots), '
        'points=[(p.x,p.y,p.z) for p in msg.pos_pts]))')
    old_geometry = "        t0 = time.monotonic()\n        mission = self.new('dynamic',"
    fixture = substitute_once(fixture, old_geometry,
        "        self.record('offset_frame_reference', dict(world=decode(ground.link_state.pose), "
        "odom=decode(self.odom.pose.pose), odom_stamp=self.odom.header.stamp.to_sec(), "
        "observed_sim=rospy.Time.now().to_sec()))\n" + old_geometry)
    fixture_path.write_text(fixture)
    original = (deploy/'run_acceptance_case.sh').read_text()
    box_driver = out/'run_box.sh'
    box_driver.write_text(substitute_once(original,'tools/deployment/validate_planning_sitl.py "$scenario"',str(fixture_path)+' "$scenario"'))
    helpers = dict(physics=contact_report, shutdown=inspect, planning=grade)
    checks = [('S04-1-offset-box','dynamic',box_driver),('S04-1-unknown','unknown',root/'F04-unknown-instrumented.sh')]
    with (root/'S04-attempts.jsonl').open('x',buffering=1) as ledger:
        record(ledger, event='batch', commit=commit, offset_y_m=0.30,
               frozen_fixture_sha256=hashlib.sha256(source.encode()).hexdigest(),
               offset_fixture_sha256=hashlib.sha256(fixture.encode()).hexdigest())
        for label,scenario,driver in checks:
            case = root/'flights'/label
            if case.exists() or (label!=checks[0][0] and not ground(entry, env)['passed']):
                raise ValueError('A unique identifier and live ground verification are required')
            record(ledger,event='start',label=label,scenario=scenario,commit=commit)
            with (root/(label+'.log')).open('x') as console:
                p=subprocess.run(['bash',str(driver),label,scenario,'2.4','1.6','0','0'],env=env,stdout=console,stderr=subprocess.STDOUT)
            safe=ground(entry,env)
            write_new(root/(label+'-ground.json'),safe)
            audit=grade_case(case,scenario,deploy,helpers,commit)
            if scenario=='unknown':
                recovery=successful_recovery(json.loads((case/'result/trace.json').read_text()))
                audit['original_failed']=list(audit['failed'])
                audit['recovery_before_commanded_landing']=recovery
                audit['checks']['fresh_after_recovery/terminal']=recovery['passed']
                audit['failed']=[k for k,ok in audit['checks'].items() if not ok]
                audit['passed']=not audit['failed']
            write_new(root/(label+'-audit.json'),audit)
            record(ledger,event='finish',label=label,scenario=scenario,exit_code=p.returncode,
                   passed=audit['passed'] and p.returncode==0 and safe['passed'],ground_verified=safe['passed'],audit=audit)
            if not safe['passed']:
                raise RuntimeError('End ground state not confirmed')


if __name__=='__main__':
    main()
