#!/usr/bin/env python3
"""Run sequential search flights on one FCU session using deterministic detections."""
import argparse,json,math,os,threading,time
from pathlib import Path
import rospy
from gazebo_msgs.srv import GetLinkState
from std_msgs.msg import String
from navigation_core.contracts import DetectionResult,RequestKey
from navigation_core.yaw import angle_delta
from validate_segments import Trial,decode

class ContinuousTrial(Trial):
    def __init__(self,args):
        self.world=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        super().__init__(args)
        self.world_done=threading.Event()
        self.delayed_once=False;self.delay_report=None;self.completed_rounds=[]
        threading.Thread(target=self.watch_world,daemon=True).start()
    def watch_world(self):
        while not self.world_done.is_set() and not rospy.is_shutdown():
            try:
                r=self.world('iris_depth_camera::iris::base_link','world')
                if r.success:
                    p=r.link_state.pose.position
                    self.record('world',dict(position=[p.x,p.y,p.z]))
            except rospy.ServiceException:pass
            time.sleep(.12)
    def detect_cb(self,msg):
        d=json.loads(msg.data);scenario=self.scenarios.get(d['mission_id'])
        if not scenario or not self.odom:return
        point=scenario['target'];p,yaw=decode(self.odom.pose.pose)
        visible=abs(angle_delta(math.atan2(point[1]-p[1],point[0]-p[0]),yaw))<math.radians(35.)
        capture=self.odom.header.stamp.to_sec()
        self.record('fixture_capture',dict(mission_id=d['mission_id'],request_id=d['request_id'],
            stop_confirmed=self.flight.get('stop_confirmed'),floor=d['capture_after'],
            measured_yaw=yaw,scan_offset_deg=self.mission.get('scan_offset_deg',0)))
        delay=self.args.initial_delay if not self.delayed_once else .15
        self.delayed_once=True;wait_started=time.monotonic()
        if delay>1.:
            (self.args.out/'delay-started.json').write_text(json.dumps(dict(
                wall_start=wait_started,request_id=d['request_id'],delay_s=delay))+'\n')
        def respond():
            time.sleep(delay)
            if delay>1.:
                world=[x['position'] for tm,k,x in self.trace if wait_started+.3<=tm<time.monotonic() and k=='world']
                feedback=[x for tm,k,x in self.trace if wait_started+.3<=tm<time.monotonic() and k=='flight']
                drift=max((math.dist(x,world[0]) for x in world),default=float('inf'))
                self.delay_report=dict(delay_s=delay,world_samples=len(world),world_drift_m=drift,
                  passed=bool(world and feedback) and drift<.30 and all(x['mode']=='HOLD' and x['context'] is None for x in feedback))
                (self.args.out/'delay-hold.json').write_text(json.dumps(self.delay_report,indent=2)+'\n')
            result=DetectionResult(RequestKey(d['mission_id'],d['request_id']),self.args.target,
                'SUCCEEDED' if visible else 'FAILED',capture,rospy.Time.now().to_sec(),
                point if visible else None,'map','' if visible else 'target_not_found')
            self.detection.publish(String(data=result.to_json()))
        threading.Thread(target=respond,daemon=True).start()
    def run_rounds(self,out):
        self.initial();flight_session=self.flight['flight_session_id']
        mission_session=self.mission['mission_session_id'];rows=self.completed_rounds
        for i,bearing in enumerate((103.,-103.,-54.),1):
            self.initial();trace_start=len(self.trace)
            assert self.flight['flight_session_id']==flight_session
            assert self.mission['mission_session_id']==mission_session
            p,yaw=decode(self.odom.pose.pose)
            theta=yaw+math.radians(bearing)
            point=(p[0]+3.6*math.cos(theta),p[1]+3.6*math.sin(theta),.6)
            assert -5.9<=point[0]<=5.9 and -3.9<=point[1]<=3.9,('fixture target outside mission area',point)
            start_world=self.world('iris_depth_camera::iris::base_link','world').link_state.pose.position
            started=time.monotonic();name=self.new('continuous_'+str(i),takeoff=True)
            self.scenarios[name]['target']=point
            result=self.wait_phase(name,('DONE','FAILED'),200.)
            assert result['phase']=='DONE',result
            if self.args.initial_delay>1.:assert self.delay_report and self.delay_report['passed'],self.delay_report
            assert result['segment_count']>0,result
            end,_=decode(self.odom.pose.pose);distance=math.dist(end[:2],point[:2])
            assert 2.05<=distance<=2.75,distance
            reached=time.monotonic();time.sleep(10.)
            held=[d for t,k,d in self.trace if t>=reached and k=='odom']
            drift=max(math.dist(d['position'],held[0]['position']) for d in held)
            assert drift<.30,drift
            world_held=[d['position'] for t,k,d in self.trace if t>=reached and k=='world']
            world_drift=max(math.dist(q,world_held[0]) for q in world_held)
            assert world_drift<.30,world_drift
            self.stop_and_land()
            assert not self.state.armed and self.landed.landed_state==1
            captures=[d for _,k,d in self.trace[trace_start:] if k=='fixture_capture' and d['mission_id']==name]
            assert captures and all(d['stop_confirmed'] for d in captures)
            scans=[d['scan_offset_deg'] for _,k,d in self.trace[trace_start:] if k=='mission' and
                   d.get('mission_id')==name and d['phase']=='SCAN_TURN']
            order=[45,-45,90,-90]
            assert scans==order[:len(scans)] and scans,scans
            world=[d['position'] for _,k,d in self.trace[trace_start:] if k=='world']
            assert world
            motion=max(math.dist(q[:2],(start_world.x,start_world.y)) for q in world)
            assert motion>.5,motion
            row=dict(round=i,passed=True,mission_id=name,bearing_deg=bearing,scans=scans,
                segments=result['segment_count'],distance_m=distance,hold_drift_m=drift,world_hold_drift_m=world_drift,
                physical_horizontal_motion_m=motion,elapsed_s=time.monotonic()-started,
                flight_session=flight_session,mission_session=mission_session,landed_disarmed=True,
                detection='deterministic response fixture, not cloud')
            rows.append(row)
            (out/('round-'+str(i)+'.json')).write_text(json.dumps(row,indent=2)+'\n')
            (out/('round-'+str(i)+'-trace.json')).write_text(json.dumps(self.trace[trace_start:])+'\n')
            print('PASS',json.dumps(row),flush=True)
        return rows

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--initial-delay',type=float,default=0.15);args=ap.parse_args()
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11311':raise RuntimeError('SITL master required')
    args.mock=False;args.live=False;args.yaw=None;args.target='deterministic red-pole fixture'
    args.out.mkdir(parents=True,exist_ok=True)
    rospy.init_node('continuous_sitl_validation',anonymous=True)
    trial=ContinuousTrial(args);error=None;rows=[]
    try:rows=trial.run_rounds(args.out)
    except Exception as exc:error=repr(exc);print('FAIL',error,flush=True)
    finally:
        try:trial.stop_and_land()
        except Exception as exc:error=(error or '')+' cleanup '+repr(exc)
        trial.world_done.set()
        (args.out/'results.json').write_text(json.dumps(dict(passed=error is None,rounds=trial.completed_rounds,error=error,delay_hold=trial.delay_report,
            final_armed=trial.state.armed if trial.state else None),indent=2)+'\n')
        (args.out/'trace.json').write_text(json.dumps(trial.trace)+'\n')
    raise SystemExit(error is not None)
if __name__=='__main__':main()
