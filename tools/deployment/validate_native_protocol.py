#!/usr/bin/python3
"""Read-only real-model coordinate audit: resized RGB-D, negatives, partial rods."""
import argparse,json,os,time
from collections import Counter
from pathlib import Path
import cv2,numpy as np,rospy
from sensor_msgs.msg import CameraInfo
from qwen_vl_grounding import QwenVLGroundingDetector
from base_detector import DetectionRejected
from navigation_core.perception import grounding_payload
from validate_search_sitl import TARGET
ap=argparse.ArgumentParser();ap.add_argument("--root",type=Path,required=True);ap.add_argument("--out",type=Path,required=True)
a=ap.parse_args();rospy.init_node("native_coordinate_audit",anonymous=True)
d=QwenVLGroundingDetector(os.environ["VLM_API_KEY"],os.environ["VLM_MODEL"],35.)
items=[]
for label in ("positive","negative"):
 for distance in ("3p6","2p4","1p6"):
  folder=a.root/"step2/fixtures"/(label+"-"+distance+"-1")
  if folder.exists():items.append((folder/"rgb.png",label=="positive",folder))
for name in ("red_pole-5efd806d-4-33.216.png","red_pole-5efd806d-6-43.816.png"):
 items.append((a.root/"step8/live-front/images"/name,True,None))
rows=[];observations=[]
for path,positive,folder in items:
 orig=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB);meta=None;depth=None
 if folder:
  meta=json.loads((folder/"metadata.json").read_text());depth=np.load(folder/"depth.npy")
 for width in (320,640,1280):
  height=width*orig.shape[0]//orig.shape[1]
  rgb=cv2.resize(orig,(width,height),interpolation=cv2.INTER_NEAREST)
  final=None;feedback=""
  for attempt in range(2):
   row=dict(frame=path.name,fixture=path.parent.name,width=width,height=height,positive=positive,attempt=attempt+1)
   start=time.monotonic();raw=None
   try:
    raw=d._call_api(rgb,TARGET,feedback);box=d.parse_response_bbox(raw,width,height)
    if box:
     x1,y1,x2,y2=box;p=rgb[y1:y2,x1:x2].astype(np.float32)
     support=int(((p[:,:,0]>110)&(p[:,:,0]>p[:,:,1]*1.5)&(p[:,:,0]>p[:,:,2]*1.5)).sum())
     correct=positive and support>=3
     row.update(bbox=list(box),red_support=support)
     if depth is not None:
      scaled=cv2.resize(depth,(width,height),interpolation=cv2.INTER_NEAREST)
      u,v,z,n=d._foreground_depth(scaled[y1:y2,x1:x2])
      K=list(meta["K"]);scale=width/orig.shape[1]
      for i in (0,2,4,5):K[i]*=scale
      point=d._pixel_to_camera_optical(x1+u,y1+v,z,K)
      world=np.asarray(meta["world_from_optical"])@np.array([*point,1.])
      error=abs(float(np.linalg.norm(world[:2]-meta["target_root_position"][:2]))-.05)
      row["surface_error_m"]=error;correct=correct and error<=.15
     row["status"]="correct_positive" if correct else "false_positive_or_wrong_location"
    else:
     payload=grounding_payload(raw)
     row["status"]=("false_negative" if positive else "correct_absence") if isinstance(payload,dict) and payload.get("found") is False else "invalid_bbox_response"
   except DetectionRejected as exc:row["status"]=str(exc)
   row.update(raw_response=raw,elapsed_s=time.monotonic()-start,response_protocol=d.response_protocol)
   rows.append(row);final=row["status"]
   print(json.dumps({k:v for k,v in row.items() if k!="raw_response"}),flush=True)
   a.out.write_text(json.dumps(dict(complete=False,rows=rows,observations=observations),indent=2,ensure_ascii=False)+"\n")
   if final not in ("invalid_bbox_response","invalid_api_response"):break
   feedback=final
  observations.append(dict(frame=str(path),width=width,final_status=final,passed=final in ("correct_positive","correct_absence")))
report=dict(complete=True,passed=all(x["passed"] for x in observations),api_calls=len(rows),observations=observations,rows=rows,totals=dict(Counter(x["status"] for x in rows)),flight_commands_issued=False,protocol=d.response_protocol)
a.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n")
raise SystemExit(not report["passed"])
