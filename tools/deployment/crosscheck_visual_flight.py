#!/usr/bin/python3
"""Independent acceptance audit using mission traces and Gazebo truth/contacts."""
import argparse,json,math,re
from pathlib import Path

def audit(case,nominal,expected):
    checks=[];errors=[];metrics={}
    def check(name,passed,**details):
        checks.append(dict(name=name,passed=bool(passed),**details))
        if not passed:errors.append(name)
    def load(name):
        path=case/name
        if not path.exists():raise ValueError("missing evidence: "+name)
        return json.loads(path.read_text())
    try:
        result=load("result/results.json");trace=load("result/trace.json")
        clearance=load("clearance.json");physics=load("physics-contacts.json");moving=load("moving.json")
        observations=[x for x in result["checks"] if x["name"]=="attempt_observations"]
        if not observations:raise ValueError("no actual mission observations")
        observed=observations[-1];metrics.update(observed)
        check("mission_completed",result["passed"] and observed["terminal"]=="DONE",reason=result.get("error"))
        check("landed_disarmed",result["final_armed"] is False and any(x["name"]=="landed_disarmed" for x in result["checks"]))
        check("actual_terminal_standoff",abs(observed["distance_at_terminal_m"]-nominal)<=.20,
              nominal_m=nominal,actual_m=observed["distance_at_terminal_m"],maximum_error_m=.20)
        scans=observed["scans"];order=[45,-45,90,-90]
        check("requested_search_direction",scans==order[:len(scans)] and (expected==0 or expected in scans),actual=scans,expected_offset=expected)
        check("bounded_detection_requests",observed["detection_requests"]<=12,count=observed["detection_requests"])
        check("physical_zero_target_contacts",physics.get("passed") and not physics.get("iris_obstacle_contacts"),
              positive_control_contacts=physics.get("positive_control_contacts"),
              actual_contacts=physics.get("iris_obstacle_contacts"))
        check("physical_logger_positive_control",physics.get("positive_control_contacts",0)>0 and physics.get("physics_messages",0)>200)
        pairs=re.findall(r'collision1:\s*"([^"]+)"\s+collision2:\s*"([^"]+)"',(case/"physics-contacts.txt").read_text())
        permitted={"ground_plane::link::collision","landing_pad::link::pad_collision"}
        unexpected=sorted({y if "iris_depth_camera" in x else x for x,y in pairs
                           if "iris_depth_camera" in x+y and
                           (y if "iris_depth_camera" in x else x) not in permitted})
        check("no_unexpected_world_contacts",not unexpected,unexpected_collision_names=unexpected,
              permitted_ground_contacts=sorted(permitted))
        localization=next((x.get("samples",[]) for x in result["checks"]
                           if x["name"]=="historical_tf_gazebo_and_target_surface"),[])
        latest_surface=localization[-1]["target_surface_error_m"] if localization else None
        check("final_observation_matches_true_target",latest_surface is not None and latest_surface<=.20,
              final_surface_error_m=latest_surface)
        metrics["semantic_outliers_diagnostic"]=sum(x["target_surface_error_m"]>.20 for x in localization)

        odoms=[(tm,data) for tm,kind,data in trace if kind=="odom"]
        check("physical_logging_covers_flight",bool(odoms) and physics["start_stamp"]<=odoms[0][1]["stamp"]+.5 and
              physics["last_message_stamp"]>=odoms[-1][1]["stamp"]-.3,
              logger_end=physics["last_message_stamp"],last_odom=odoms[-1][1]["stamp"] if odoms else None)
        check("moving_rgbd",moving.get("passed",False),frames=moving.get("count"),
              map_p95_m=moving.get("map_surface_error_p95_of_frame_p95_m"))
        captures=[(tm,data) for tm,kind,data in trace if kind=="search_capture" and data["mission_id"]==observed["mission_id"]]
        replies=[(tm,data) for tm,kind,data in trace if kind=="detect_result" and data["mission_id"]==observed["mission_id"]]
        capture_by_id={data["request_id"]:data for _,data in captures}
        check("stable_fresh_capture",bool(captures) and all(data.get("stop_confirmed") for _,data in captures))
        matched=[]
        for _,reply in replies:
            capture=capture_by_id.get(reply["request_id"])
            matched.append(bool(capture) and reply["capture_stamp"]>=capture["floor"]-.001 and
                           0<=reply["completed_stamp"]-reply["capture_stamp"]<=42.1)
        check("reply_ids_and_capture_times",bool(matched) and all(matched),replies=len(matched))
        done=[tm for tm,kind,data in trace if kind=="mission" and data.get("mission_id")==observed["mission_id"] and data["phase"]=="DONE"]
        if done and odoms:
            done_time=done[0];done_stamp=min(odoms,key=lambda x:abs(x[0]-done_time))[1]["stamp"]
            held=[x for x in clearance["samples"] if done_stamp+.5<=x["stamp"]<=done_stamp+9.5]
            drift=max((math.dist(x["world_xyz"],held[0]["world_xyz"]) for x in held),default=math.inf)
            span=held[-1]["stamp"]-held[0]["stamp"] if held else 0
            band=max((abs(x["horizontal_distance_m"]-nominal) for x in held),default=math.inf)
            hold_check=any(x["name"]=="observed_and_held" and x.get("hold_seconds",0)>=10 for x in result["checks"])
            check("ten_second_hold_gazebo_truth",hold_check and len(held)>=70 and span>=8.4 and drift<=.30 and band<=.20,
                  samples=len(held),covered_core_seconds=span,world_drift_m=drift,maximum_standoff_error_m=band)
            metrics.update(world_hold_drift_m=drift,world_hold_standoff_error_m=band)
            flight=[data for tm,kind,data in trace if kind=="flight" and done_time+.5<=tm<=done_time+9.5]
            check("no_trajectory_revival_after_done",bool(flight) and all(data["mode"]=="HOLD" and data.get("context") is None for data in flight))
        else:check("ten_second_hold_gazebo_truth",False,reason="no completed mission")
        minimum=clearance.get("minimum_conservative_clearance_m")
        check("conservative_rod_clearance",minimum is not None and minimum>.5,minimum_m=minimum,
              scope="central rod bound; complete tripod covered separately by physics contact logger")
        metrics["minimum_conservative_rod_clearance_m"]=minimum
    except Exception as exc:errors.append(str(exc))
    report=dict(passed=not errors,case=str(case),nominal_standoff_m=nominal,expected_scan_offset=expected,
                checks=checks,errors=errors,metrics=metrics,
                scope="fixed project SITL, real cloud; no claim for deferred semantic/visibility defects")
    (case/"acceptance.json").write_text(json.dumps(report,indent=2)+"\n")
    return report

def main():
    p=argparse.ArgumentParser();p.add_argument("--case",type=Path,required=True)
    p.add_argument("--standoff",type=float,required=True);p.add_argument("--expected",type=int,required=True)
    a=p.parse_args();r=audit(a.case,a.standoff,a.expected)
    print(json.dumps({k:v for k,v in r.items() if k!="checks"}),flush=True)
    raise SystemExit(not r["passed"])
if __name__=="__main__":main()
