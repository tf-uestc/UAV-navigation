"""Replay cross-topic delivery order against a selected gateway file without a ROS master."""
import argparse
import importlib.util
import json
from pathlib import Path
import threading
from types import SimpleNamespace


class Publisher:
    def __init__(self):
        self.rows=[]

    def publish(self,msg):
        self.rows.append(json.loads(msg.data))


def replay(module, order):
    node=module.ManagedPlanner.__new__(module.ManagedPlanner)
    node.lock=threading.RLock()
    node.session='planner-a'
    node.gateway='gateway'
    node.active=dict(key=('mission',1),accepted=True)
    node.retired_sessions=set()
    node.cancelling={}
    node.result_pub=Publisher()
    node.disposition_pub=Publisher()
    forwarded=[]
    node.forward=lambda packet: forwarded.append(packet.traj_id)
    packet=SimpleNamespace(mission_id='mission',goal_id=1,traj_id=2,planner_session_id='planner-a',emergency_stop=True)
    status=SimpleNamespace(mission_id='mission',goal_id=1,planner_session_id='planner-a',
                           status='FAILED',reason='emergency_stop_requires_new_goal',map_ready=True)
    if order=='status_first':
        node.on_status(status)
    if order=='retired_epoch':
        packet.planner_session_id='planner-zero'
    if order=='other_goal':
        node.active['key']=('new-mission',1)
    node.on_spline(packet)
    if order=='spline_first':
        node.on_status(status)
    rejected=node.disposition_pub.rows
    expected='spline_first'==order
    if expected:
        success=(forwarded==[2] and not rejected)
    else:
        success=(not forwarded and len(rejected)==1 and rejected[0]['accepted'] is False
                 and rejected[0]['traj_id']==2 and rejected[0]['goal_id']==1
                 and rejected[0]['mission_id']=='mission' and rejected[0]['planner_session_id']==packet.planner_session_id)
    return dict(order=order,passed=success,forwarded=forwarded,gateway_rejections=rejected)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    args=parser.parse_args()
    spec=importlib.util.spec_from_file_location('gateway_under_test',str(args.source))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows=[replay(module,x) for x in ('spline_first','status_first','retired_epoch','other_goal')]
    print(json.dumps(dict(passed=all(row['passed'] for row in rows),rows=rows),indent=2))
    raise SystemExit(not all(row['passed'] for row in rows))
