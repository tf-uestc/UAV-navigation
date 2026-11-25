"""Check archived landing results against the individual ROS events."""
import argparse
import hashlib
import json
import tarfile
from pathlib import Path


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('archive_dir', type=Path, help='Directory holding T05-evidence.tar.gz')
args = parser.parse_args()
base = args.archive_dir
dest = Path(__file__).resolve().parent.parent / 'landing-followup'
manifest = json.loads((dest / 'T05-evidence-manifest.json').read_text())
archive = base / manifest['file']
assert archive.stat().st_size == manifest['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['sha256']
results = []
with tarfile.open(archive) as tar:
    ledger = [json.loads(line) for line in tar.extractfile('T05-attempts.jsonl')]
    starts = [r['label'] for r in ledger if r['event'] == 'start']
    ends = [r for r in ledger if r['event'] == 'finish']
    assert len(starts) == len(set(starts)) == 4 and starts == [r['label'] for r in ends]
    for row in ends:
        label = row['label']
        trace = json.load(tar.extractfile('flights/'+label+'/landing-handshake.json'))
        events = trace['events']
        land = min(t for t,k,d in events if k=='command' and d['command']=='LAND')
        disarm = min(t for t,k,d in events if t>=land and k=='fcu' and d['connected'] and not d['armed'])
        complete = [t for t,k,d in events if t>=land and k=='flight' and d['mode']=='COMPLETE' and d['owns_flight'] is False]
        mission = [t for t,k,d in events if t>=land and k=='mission' and d['phase']=='LANDED']
        failed = {d['reason'] for t,k,d in events if t>=land and k=='flight' and d['mode']=='FAILED'}
        until = min(complete,default=float('inf'))
        stamps = {d['stamp'] for t,k,d in events if disarm<=t<=until and k=='extended' and d['landed_state']==1}
        speeds = [d['speed'] for t,k,d in events if disarm<=t<=until and k=='odom']
        item = dict(label=label,passed=row['passed'],driver_exit=row['exit_code'],
                    primary=row['audit']['primary'],physics_contacts=row['audit']['contact_stream'],
                    postflight_ground=row['ground_verified'],handshake=trace['checks'],
                    disarm_s=disarm-land,complete_s=until-land if complete else None,
                    mission_landed_s=min(mission)-land if mission else None,failures=sorted(failed),
                    distinct_ground_reports_before_complete=len(stamps),
                    samples_over_point15_before_complete=sum(v>.15 for v in speeds),
                    odom_samples_before_complete=len(speeds),after_disarm_over_point15=trace['odom_samples_over_point15'],
                    max_speed_after_disarm=trace['max_reported_speed_after_disarm'])
        assert item['passed'] == (item['primary'] and item['physics_contacts'] and item['postflight_ground'] and all(trace['checks'].values()) and bool(complete) and bool(mission) and not failed)
        results.append(item)
(dest/'T05-independent.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
for result in results:
    print(json.dumps(result))
