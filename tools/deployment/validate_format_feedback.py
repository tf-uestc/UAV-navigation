#!/usr/bin/python3
"""Real API protocol audit of previously failed RGB frames; no flight commands."""
import argparse,json,os,time
from pathlib import Path
from collections import Counter
import cv2,numpy as np,rospy
from qwen_vl_grounding import QwenVLGroundingDetector
from navigation_core.perception import grounding_payload
from validate_search_sitl import TARGET
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
ap.add_argument('--force-feedback',action='store_true')
ap.add_argument('--positive-frames',nargs='+',type=Path)
a=ap.parse_args();rospy.init_node('format_recovery_audit',anonymous=True)
d=QwenVLGroundingDetector(os.environ['VLM_API_KEY'],os.environ['VLM_MODEL'],35.)
folder=a.root/'step4/front-2p4-aligned/images'
obs=json.loads((folder/'observations.json').read_text())['rows']
items=[(p,True) for p in a.positive_frames] if a.positive_frames else [(folder/x['exact_frame'],True) for x in obs[:2]]
items += [(a.root/'step2/fixtures'/('negative-3p6-'+str(n))/'rgb.png',False) for n in (1,2)]
rows=[];results=[]
for repeat in range(3):
 for path,positive in items:
  rgb=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB);outcome=None
  for attempt in range(1 if a.force_feedback else 2):
   feedback='' if attempt==0 and not a.force_feedback else 'invalid_bbox_response';started=time.monotonic()
   raw=d._call_api(rgb,TARGET,feedback)
   box=d.parse_response_bbox(raw,rgb.shape[1],rgb.shape[0])
   support=0
   if box:
    x1,y1,x2,y2=box;p=rgb[y1:y2,x1:x2].astype(np.float32)
    support=int(((p[:,:,0]>110)&(p[:,:,0]>p[:,:,1]*1.5)&(p[:,:,0]>p[:,:,2]*1.5)).sum())
    outcome='correct_positive' if positive and support>=8 else 'false_positive_or_wrong_bbox'
   else:
    payload=grounding_payload(raw)
    outcome=('false_negative' if positive else 'correct_absence') if isinstance(payload,dict) and payload.get('found') is False else 'invalid_bbox_response'
   row=dict(frame=path.name,positive=positive,repeat=repeat+1,attempt=attempt+1,
     feedback=feedback,status=outcome,red_support=support,elapsed_s=time.monotonic()-started,raw_response=raw)
   rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='raw_response'}),flush=True)
   if outcome!='invalid_bbox_response':break
   time.sleep(.65)
  results.append(dict(frame=path.name,repeat=repeat+1,passed=outcome in ('correct_positive','correct_absence')))
  a.out.write_text(json.dumps(dict(complete=False,rows=rows,observations=results),indent=2,ensure_ascii=False)+'\n')
report=dict(forced_feedback=a.force_feedback,complete=True,passed=all(x['passed'] for x in results),rows=rows,observations=results,
 totals=dict(Counter(x['status'] for x in rows)),api_calls=len(rows),flight_commands_issued=False,
 scope='Actual RGB frames and strict protocol plus red-pixel support; depth localization separately verified.')
a.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
raise SystemExit(not report['passed'])
