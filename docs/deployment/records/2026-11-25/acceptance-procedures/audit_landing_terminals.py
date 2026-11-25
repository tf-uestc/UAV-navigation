"""Read all physical landing and controller terminal events after the first LAND in a raw batch."""
import json
from pathlib import Path
import sys
import tarfile


def inspect(trace):
    lands=[t for t,k,d in trace if k=='flight_command' and d.get('command')=='LAND']
    if not lands:
        return dict(land_command_observed=False)
    start=min(lands)
    after=[(t-start,k,d) for t,k,d in trace if t>=start]
    execs=[(t,d) for t,k,d in after if k=='flight']
    fcu=[(t,d) for t,k,d in after if k=='fcu']
    missions=[(t,d) for t,k,d in after if k=='mission']
    transitions=[]
    for t,d in execs:
        key=(d.get('mode'), d.get('reason'), d.get('owns_flight'))
        if not transitions or tuple(transitions[-1][1:])!=key:
            transitions.append([round(t,6),*key])
    return dict(land_command_observed=True, controller_transitions=transitions,
                controller_complete=any(d.get('mode')=='COMPLETE' for _,d in execs),
                controller_timeout=any(d.get('mode')=='FAILED' and d.get('reason')=='landing_timeout' for _,d in execs),
                mission_landed=any(d.get('phase')=='LANDED' for _,d in missions),
                last_mission_phase=missions[-1][1].get('phase') if missions else None,
                first_disarmed_wall_s=next((round(t,6) for t,d in fcu if d.get('armed') is False),None),
                last_fcu_armed=fcu[-1][1].get('armed') if fcu else None)


def main():
    reports={}
    for path in map(Path, sys.argv[2:]):
        batch=path.name.split('-')[0]
        rows={}
        with tarfile.open(path,'r|gz') as archive:
            for member in archive:
                if member.name.startswith('flights/') and member.name.endswith('/result/trace.json'):
                    label=member.name.split('/')[1]
                    rows[label]=inspect(json.load(archive.extractfile(member)))
        reports[batch]=dict(count=len(rows),controller_timeouts=[k for k,v in rows.items() if v.get('controller_timeout')],
                            complete=sum(v.get('controller_complete',False) for v in rows.values()),
                            mission_landed=sum(v.get('mission_landed',False) for v in rows.values()), cases=rows)
    Path(sys.argv[1]).write_text(json.dumps(reports,indent=2)+'\n',encoding='utf-8')
    for k,r in reports.items():
        print(k,r['count'],r['controller_timeouts'],r['complete'],r['mission_landed'])


if __name__=='__main__':
    main()
