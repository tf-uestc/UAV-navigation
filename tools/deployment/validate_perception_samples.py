#!/usr/bin/env python3
"""Audit a fixed RGB-D dataset through the real model without flight outputs."""
import argparse,hashlib,importlib.util,json,os,time
from collections import Counter
from pathlib import Path
import cv2,numpy as np,rospy
from sensor_msgs.msg import CameraInfo
from base_detector import DetectionRejected
from qwen_vl_grounding import QwenVLGroundingDetector
from validate_search_sitl import TARGET

def absent_in_truth(meta):
 inv=np.linalg.inv(np.asarray(meta['world_from_optical']))
 x,y,z=meta['target_root_position']
 corners=np.array([[x+dx,y+dy,z+dz,1.] for dx in (-.05,.05) for dy in (-.05,.05) for dz in (0.,.9)]).T
 points=inv@corners
 if np.all(points[2]<=0):return True
 if np.any(points[2]<=0):return False
 K=meta['K'];u=K[0]*points[0]/points[2]+K[2];v=K[4]*points[1]/points[2]+K[5]
 return u.max()<-4 or u.min()>meta['width']+4 or v.max()<-4 or v.min()>meta['height']+4

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True)
 ap.add_argument('--out',type=Path,required=True);ap.add_argument('--baseline',action='store_true')
 ap.add_argument('--repeats',type=int,default=2);args=ap.parse_args()
 rospy.init_node('perception_dataset_audit',anonymous=True);args.out.parent.mkdir(parents=True,exist_ok=True)
 root=args.root;manifest=root/'step2/dataset.json'
 if manifest.exists():items=json.loads(manifest.read_text())
 else:
  items=[]
  for p in sorted((root/'step2/fixtures').glob('*/metadata.json')):
   m=json.loads(p.read_text());items.append(dict(path=str(p.parent),label=m['label']))
  negatives=sum(x['label']=='negative' for x in items)
  for p in sorted((root/'fixtures/passive').glob('negative-*/metadata.json')):
   if negatives>=6:break
   m=json.loads(p.read_text())
   if absent_in_truth(m):
    items.append(dict(path=str(p.parent),label='negative'));negatives+=1
  assert len(items)==12 and negatives==6,items
  for x in items:x['image_sha256']=hashlib.sha256(Path(x['path'],'rgb.png').read_bytes()).hexdigest()
  manifest.write_text(json.dumps(items,indent=2)+'\n')
 cls=QwenVLGroundingDetector
 if args.baseline:
  path=Path('/root/autodl-tmp/uav/validation/search-acceptance-2026-11-20-7jhF/UAV-navigation/ws/src/uav_vln_bringup/scripts/qwen_vl_grounding.py')
  spec=importlib.util.spec_from_file_location('baseline_detector_audit',str(path))
  module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);cls=module.QwenVLGroundingDetector
 detector=cls(api_key=os.environ['VLM_API_KEY'],model=os.environ['VLM_MODEL'],timeout=35.)
 raw_reply=[None];original=detector._call_api
 def audit_call(rgb,target):
  raw_reply[0]=original(rgb,target);return raw_reply[0]
 detector._call_api=audit_call
 rows=[];observations=[]
 retryable={'invalid_bbox_response','api_timeout','api_connect_timeout','api_network_error','api_http_429','api_http_502','api_http_503','api_http_504'}
 for repeat in range(args.repeats):
  for item in items:
   folder=Path(item['path']);meta=json.loads((folder/'metadata.json').read_text())
   rgb=cv2.cvtColor(cv2.imread(str(folder/'rgb.png')),cv2.COLOR_BGR2RGB)
   depth=np.load(folder/'depth.npy')
   info=CameraInfo(width=meta['width'],height=meta['height'],K=meta['K'])
   outcome=None
   for attempt in range(1 if args.baseline else 2):
    raw_reply[0]=None;start=time.monotonic()
    row=dict(frame=folder.name,label=item['label'],repeat=repeat+1,attempt=attempt+1)
    try:
     d=detector.detect(rgb,depth,TARGET,info)
     x1,y1,x2,y2=d.bbox;patch=rgb[y1:y2,x1:x2].astype(np.float32)
     red=(patch[:,:,0]>110)&(patch[:,:,0]>patch[:,:,1]*1.5)&(patch[:,:,0]>patch[:,:,2]*1.5)
     support=int(red.sum())
     point=np.asarray(meta['world_from_optical'])@np.array([*d.point_camera,1.])
     target=meta['target_root_position'];surface_error=abs(float(np.linalg.norm(point[:2]-target[:2]))-.05)
     correct=item['label']=='positive' and support>=8 and surface_error<=.15
     row.update(status='correct_positive' if correct else 'false_positive_or_wrong_location',
       bbox=list(d.bbox),red_support=support,surface_error_m=surface_error)
    except DetectionRejected as exc:
     reason=str(exc)
     row.update(status='correct_absence' if reason=='target_not_found' and item['label']=='negative'
       else 'false_negative' if reason=='target_not_found' else reason)
    row.update(elapsed_s=round(time.monotonic()-start,3),raw_response=raw_reply[0])
    rows.append(row);outcome=row['status']
    totals=dict(Counter(x['status'] for x in rows))
    result=dict(mode='baseline' if args.baseline else 'candidate',complete=False,passed=False,
      frames=len(items),repeats=args.repeats,rows=rows,observations=observations,totals=totals,
      flight_commands_issued=False)
    args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in row.items() if k!='raw_response'}),flush=True)
    if row['status'] not in retryable:break
    time.sleep(.65)
   observations.append(dict(frame=folder.name,label=item['label'],repeat=repeat+1,
       final_status=outcome,passed=outcome in ('correct_positive','correct_absence')))
   time.sleep(.25)
 result=dict(mode='baseline' if args.baseline else 'candidate',complete=True,
   passed=all(x['passed'] for x in observations),frames=len(items),repeats=args.repeats,
   rows=rows,observations=observations,totals=dict(Counter(x['status'] for x in rows)),flight_commands_issued=False)
 args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
 print('COMPLETE',json.dumps({k:v for k,v in result.items() if k not in ('rows','observations')}),flush=True)
 raise SystemExit(not result['passed'] and not args.baseline)
if __name__=='__main__':main()
