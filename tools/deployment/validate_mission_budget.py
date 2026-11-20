#!/usr/bin/env python3
"""Check configured mission deadlines and the unchanged per-request lifetime."""
import argparse,importlib.util,json,time
from pathlib import Path
import rospy
from navigation_core.contracts import DetectionResult,RequestKey
from navigation_core.lifecycle import CallBudget,RequestRegistry

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    source=Path(__file__).resolve().parents[2]/'ws/src/uav_vln_bringup/scripts/segment_mission.py'
    spec=importlib.util.spec_from_file_location('budget_segment',str(source))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    checks=[]
    rospy.init_node('mission_budget_validation',anonymous=True)
    def case(name,limit,elapsed,expect_failure):
        obj=object.__new__(module.SegmentMission)
        obj.phase='OBSERVE_MAP';obj.started=0.;obj.mission_timeout=limit
        obj.deadline=1e6;obj.last_ros=None;obj.flight_session='test-flight'
        obj.settled_at=rospy.Time.now().to_sec()
        obj.flight_status=lambda:dict(flight_session_id='test-flight',owns_flight=True,mode='HOLD')
        calls=[];obj.brake=lambda *a:calls.append(a)
        obj.tick(elapsed)
        assert bool(calls)==expect_failure,(name,calls)
        if expect_failure:assert calls==[('FAILED','mission_deadline')],calls
        checks.append(dict(name=name,passed=True))
    case('151_seconds_not_rejected_by_old_hardcoded_limit',720.,151.,False)
    case('configured_720_second_deadline_stops_mission',720.,720.1,True)
    case('custom_deadline_is_honored',60.,60.1,True)
    b=CallBudget(max_calls=12,max_seconds=720.);b.begin(0.)
    assert all(b.reserve(i*42.) for i in range(12))
    assert not b.reserve(510.)
    checks.append(dict(name='twelve_call_ceiling_is_enforced',passed=True))
    b.begin(0.);assert not b.reserve(720.)
    checks.append(dict(name='call_budget_uses_same_720_second_limit',passed=True))
    for elapsed,accepted in [(30.,True),(41.999,True),(42.,False)]:
        r=RequestRegistry();r.begin('clock-test');key=r.issue('red pole',0.,42.)
        result=DetectionResult(key,'red pole','SUCCEEDED',100.,100.+elapsed,(3.,0.,1.6),'map','')
        assert r.resolve(result,elapsed,ros_now=100.+elapsed,max_age=42.) is accepted
        checks.append(dict(name='request_age_'+str(elapsed),passed=True,accepted=accepted))
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(dict(passed=True,checks=checks),indent=2)+'\n')
    print(json.dumps(dict(passed=True,checks=checks)))
if __name__=='__main__':main()
