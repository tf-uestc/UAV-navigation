#!/usr/bin/python3
"""Record contacts through a whole acceptance flight, including landing."""
import argparse,json,signal,time
from pathlib import Path
import rospy
from observe_physics_contacts import PhysicsContacts
ap=argparse.ArgumentParser();ap.add_argument('--out',required=True,type=Path)
ap.add_argument('--model',default='red_tripod_A');a=ap.parse_args()
rospy.init_node('flight_physics_observer',anonymous=True,disable_signals=True)
stopped=[False]
for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *x:stopped.__setitem__(0,True))
observer=PhysicsContacts(a.out,a.model);error=None
try:
 observer.start()
 (a.out/'contacts-ready.json').write_text(json.dumps(dict(ready=True))+'\n')
 while not stopped[0] and not rospy.is_shutdown():time.sleep(.1)
except Exception as exc:error=repr(exc)
finally:
 result=observer.finish()
 if error:result.update(passed=False,error=error);(a.out/'physics-contacts.json').write_text(json.dumps(result,indent=2)+'\n')
raise SystemExit(not result['passed'])
