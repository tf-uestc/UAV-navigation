"""Run one existing SITL test and additionally observe its complete landing handshake."""
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String


def main():
    out=Path(sys.argv[1]); command=sys.argv[2:]
    rospy.init_node('landing_handshake_audit',anonymous=True,disable_signals=True)
    assert rospy.get_param('/use_sim_time') is True
    assert rospy.get_param('/mavros/fcu_url') == 'udp://:14540@127.0.0.1:14580'
    events=[]; lock=threading.Lock()
    def add(tag, payload):
        with lock:
            events.append([time.monotonic(),tag,payload])
    def status(m,tag,fields):
        msg=json.loads(m.data); add(tag,{k:msg.get(k) for k in fields})
    def odom(m):
        v=m.twist.twist.linear; p=m.pose.pose.position
        add('odom',dict(stamp=m.header.stamp.to_sec(),speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),xyz=[p.x,p.y,p.z]))
    def state(m):
        add('fcu',dict(stamp=m.header.stamp.to_sec(),connected=m.connected,armed=m.armed,mode=m.mode))
    def extended(m):
        add('extended',dict(stamp=m.header.stamp.to_sec(),landed_state=m.landed_state))
    subscriptions=[rospy.Subscriber('/uav/flight/command',String,lambda m:status(m,'command',('command','command_id')),queue_size=30),
                   rospy.Subscriber('/uav/flight/status',String,lambda m:status(m,'flight',('mode','reason','owns_flight','flight_session_id')),queue_size=30),
                   rospy.Subscriber('/uav/segment/status',String,lambda m:status(m,'mission',('mission_id','phase','reason')),queue_size=30),
                   rospy.Subscriber('/mavros/state',State,state,queue_size=30),
                   rospy.Subscriber('/mavros/extended_state',ExtendedState,extended,queue_size=30),
                   rospy.Subscriber('/mavros/local_position/odom',Odometry,odom,queue_size=60)]
    child=subprocess.run(command)
    time.sleep(4.)
    for s in subscriptions:
        s.unregister()
    with lock:
        rows=sorted(events,key=lambda r:r[0])
    lands=[t for t,k,d in rows if k=='command' and d['command']=='LAND']
    start=min(lands) if lands else math.inf
    filtered=[r for r in rows if r[0]>=start-1.]
    after=[r for r in filtered if r[0]>=start]
    released=[t for t,k,d in after if k=='flight' and d['mode']=='COMPLETE' and d['owns_flight'] is False]
    landed=[t for t,k,d in after if k=='mission' and d['phase']=='LANDED']
    disarmed=[t for t,k,d in after if k=='fcu' and d['connected'] and not d['armed']]
    floor=disarmed[0] if disarmed else math.inf
    ground=sorted({d['stamp'] for t,k,d in after if t>=floor and k=='extended' and d['landed_state']==1})
    speeds=[d['speed'] for t,k,d in after if t>=floor and k=='odom']
    failures=[d['reason'] for _,k,d in after if k=='flight' and d['mode']=='FAILED']
    checks=dict(driver=child.returncode==0,land_command=bool(lands),distinct_disarmed_ground_reports=len(ground)>=2,
                normal_controller_completion=bool(released),mission_landed=bool(landed),no_executor_failure=not failures)
    report=dict(passed=all(checks.values()),checks=checks,failed_reasons=sorted(set(failures)),
                ground_report_count=len(ground),first_disarmed_s=floor-start if disarmed else None,
                first_complete_s=released[0]-start if released else None,first_mission_landed_s=landed[0]-start if landed else None,
                max_reported_speed_after_disarm=max(speeds) if speeds else None,
                odom_samples_over_point15=sum(v>.15 for v in speeds),events=filtered)
    out.write_text(json.dumps(report,separators=(',',':'))+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='events'}),flush=True)
    return int(not report['passed'])


if __name__=='__main__':
    sys.exit(main())
