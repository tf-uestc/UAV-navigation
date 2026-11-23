#!/usr/bin/python3
"""Audit response syntax independently of deferred visual-semantic correctness."""
import argparse,importlib.util,json,os,time
from collections import Counter
from pathlib import Path
import cv2,numpy as np,rospy
from qwen_vl_grounding import QwenVLGroundingDetector
from base_detector import DetectionRejected
from validate_search_sitl import TARGET

def classify(detector,raw,width,height):
    try:return "detected",detector.parse_response(raw,width,height)
    except DetectionRejected as exc:return str(exc),None

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--previous-records",type=Path,required=True)
    ap.add_argument("--out",type=Path,required=True);ap.add_argument("--before-source",type=Path)
    ap.add_argument("--live",action="store_true");a=ap.parse_args()
    rospy.init_node("response_format_audit",anonymous=True)
    detector=QwenVLGroundingDetector(os.environ.get("VLM_API_KEY","historical-only"),os.environ.get("VLM_MODEL","qwen3.8-omni-flash"),35.)
    historical=[];fixed=0
    baseline=None
    if a.before_source:
        spec=importlib.util.spec_from_file_location("response_before_fix",str(a.before_source))
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        baseline=mod.QwenVLGroundingDetector("historical-only")
    for rel in ("step2/cloud-baseline.json","step2/cloud-candidate.json",
                "step8/partial-pole-api.json","step8/partial-pole-native.json"):
        report=json.loads((a.previous_records/rel).read_text())
        for index,row in enumerate(report["rows"]):
            raw=row.get("raw_response")
            if not isinstance(raw,str):continue
            status,box=classify(detector,raw,640,480)
            before=baseline.parse_response_bbox(raw,640,480) if baseline else None
            recovered=box is not None and baseline is not None and before is None
            fixed+=bool(recovered)
            historical.append(dict(report=rel,row=index+1,raw_response=raw,status=status,bbox=box,
                prior_format_status=row.get("status"),newly_supported=bool(recovered),
                format_passed=status in ("detected","target_not_found")))
    live=[];observations=[]
    if a.live:
        frames=[]
        for distance in ("3p6","2p4","1p6"):
            folder=a.previous_records/"step2/fixtures"/("positive-"+distance+"-1")
            for width in (320,640,1280):frames.append((folder/"rgb.png",True,width,folder))
        frames.append((a.previous_records/"step8/live-front/images/red_pole-5efd806d-4-33.216.png",True,640,None))
        for n in (1,2):
            folder=a.previous_records/"step2/fixtures"/("negative-3p6-"+str(n))
            frames.append((folder/"rgb.png",False,640,folder))
        for path,present,width,folder in frames:
            original=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
            height=width*original.shape[0]//original.shape[1]
            rgb=cv2.resize(original,(width,height),interpolation=cv2.INTER_NEAREST)
            feedback="";status="invalid_api_response"
            for attempt in range(2):
                started=time.monotonic();raw=None;box=None
                try:
                    raw=detector._call_api(rgb,TARGET,feedback)
                    status,box=classify(detector,raw,width,height)
                except DetectionRejected as exc:status=str(exc)
                row=dict(frame=str(path),width=width,height=height,attempt=attempt+1,status=status,
                    raw_response=raw,bbox=box,elapsed_s=time.monotonic()-started,
                    format_passed=status in ("detected","target_not_found"))
                if box:
                    x1,y1,x2,y2=box;patch=rgb[y1:y2,x1:x2].astype(np.float32)
                    red=int(((patch[:,:,0]>110)&(patch[:,:,0]>patch[:,:,1]*1.5)&(patch[:,:,0]>patch[:,:,2]*1.5)).sum())
                    row["red_pixel_support_diagnostic"]=red
                    row["semantic_matches_fixture_diagnostic"]=present and red>=3
                    if folder and present and red>=3:
                        meta=json.loads((folder/"metadata.json").read_text())
                        depth=cv2.resize(np.load(folder/"depth.npy"),(width,height),interpolation=cv2.INTER_NEAREST)
                        try:
                            u,v,z,n=detector._foreground_depth(depth[y1:y2,x1:x2]);K=list(meta["K"]);scale=width/original.shape[1]
                            for i in (0,2,4,5):K[i]*=scale
                            pt=detector._pixel_to_camera_optical(x1+u,y1+v,z,K)
                            world=np.asarray(meta["world_from_optical"])@np.array([*pt,1.])
                            row["target_surface_error_diagnostic_m"]=abs(float(np.linalg.norm(world[:2]-meta["target_root_position"][:2]))-.05)
                        except DetectionRejected as exc:row["depth_diagnostic"]=str(exc)
                elif status=="target_not_found":row["semantic_matches_fixture_diagnostic"]=not present
                live.append(row)
                print(json.dumps({k:v for k,v in row.items() if k!="raw_response"}),flush=True)
                a.out.write_text(json.dumps(dict(complete=False,historical=historical,live_calls=live,live_observations=observations),indent=2,ensure_ascii=False)+"\n")
                if row["format_passed"]:break
                if status not in ("invalid_bbox_response","invalid_api_response"):break
                feedback=status
            observations.append(dict(frame=str(path),width=width,passed=status in ("detected","target_not_found"),final_status=status))
    passed=all(x["format_passed"] for x in historical) and all(x["passed"] for x in observations)
    report=dict(complete=True,passed=passed,scope="response-format compatibility only; semantic false positives and visibility explicitly deferred by user",
        historical_rows=len(historical),historical_newly_supported=fixed,historical=historical,
        live_calls=live,live_observations=observations,live_api_calls=len(live),
        live_format_statuses=dict(Counter(x["status"] for x in live)),
        flight_commands_issued=False,production_color_or_visibility_changes=False)
    a.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n")
    print("COMPLETE",json.dumps({k:v for k,v in report.items() if k not in ("historical","live_calls","live_observations")}),flush=True)
    raise SystemExit(not passed)
if __name__=="__main__":main()
