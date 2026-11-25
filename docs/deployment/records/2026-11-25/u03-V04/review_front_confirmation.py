import cv2, numpy as np, json, re, math
from pathlib import Path
import sys
cv2.setNumThreads(1)
case=Path(sys.argv[1])
def read(p):return json.loads(p.read_text())
obs=read(case/'images/observations.json')['rows']
trace=read(case/'result/trace.json')
truth=read(case/'clearance.json')['samples']
poses={r['request_id']:r for _,k,r in trace if k=='search_capture' and r.get('mission_id')==obs[0]['request']['mission_id']}
raw=[]
for line in (case/'launch.log').read_text().splitlines():
 m=re.search(r'VLM response \(\d+ chars\): (.*)',line)
 if m:
  raw.append(m.group(1).split('\x1b')[0])
def red(im):
 b,g,r=np.moveaxis(im.astype(np.float32),2,0)
 return (r>110)&(r>1.5*g)&(r>1.5*b)
def geom(a,z):
 n,lab,stats,cen=cv2.connectedComponentsWithStats(a.astype('uint8'))
 components=[]
 for i in np.argsort(-stats[1:,4])[:2]+1:
  px=lab==i
  xs=z[px & np.isfinite(z)]
  xs=xs[(xs>.2)&(xs<8)]
  x,y,w,h,count=map(int,stats[i])
  components.append(dict(xyxy_inclusive=[x,y,x+w-1,y+h-1],pixels=count,valid_depth_count=len(xs),median_depth_m=float(np.median(xs)) if len(xs) else None))
 return dict(red_pixels=int(a.sum()),components=components)
rows=[]
for row in obs:
 r=row['result']; rid=r['request_id']; t=r['capture_stamp']
 im=cv2.imread(str(case/'images'/row['exact_frame'])); f=np.load(case/'images'/row['exact_depth']); z=f[f.files[0]]; z=z*.001 if z.dtype==np.uint16 else z
 ok,jpg=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,85]); ji=cv2.imdecode(jpg,cv2.IMREAD_COLOR)
 p=poses[rid]; qt=min(truth,key=lambda q:abs(q['stamp']-t))
 rows.append(dict(request=rid,raw=raw[rid-1] if rid<=len(raw) else None,outcome={k:r[k] for k in ('capture_stamp','completed_stamp','status','reason','point')},image=row['exact_frame'],depth=row['exact_depth'],shape=im.shape,raw_rgb=geom(red(im),z),jpeg85=geom(red(ji),z),search_pose=p,truth={k:qt.get(k) for k in ('world_xyz','world_yaw','world_speed_mps','stamp','horizontal_distance_m')}))
print(json.dumps(dict(rows=rows,cloud_logged_count=len(raw),red_rule='R>110, R>1.5*G, R>1.5*B, 8-connected; xyxy inclusive',depth_bounds_m=[.2,8]),ensure_ascii=True,indent=2))

