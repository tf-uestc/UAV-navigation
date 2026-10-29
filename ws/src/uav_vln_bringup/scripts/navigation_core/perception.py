"""Explicit image/coordinate contracts; no model/network calls."""
import json
import math
import re
from .contracts import finite

def parse_bbox(text,width,height,coordinate_system="pixel",allow_legacy=False):
    if type(width) is not int or type(height) is not int or min(width,height)<1:
        raise ValueError("invalid image size")
    if coordinate_system not in ("pixel","normalized_1000"):
        raise ValueError("unsupported coordinate protocol")
    if not isinstance(text,str): return None
    cleaned=text.strip()
    if cleaned.startswith("```") and cleaned.endswith("```"):
        cleaned=re.sub(r"^\`\`\`(?:json)?\s*","",cleaned)[:-3].strip()
    try:
        data=json.loads(cleaned)
    except (ValueError,TypeError):
        if not allow_legacy: return None
        match=re.fullmatch(r"(?:<box>\s*)?\[?\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]?(?:\s*</box>)?",cleaned)
        if not match: return None
        data=dict(found=True,bbox=[int(x) for x in match.groups()],coordinate_system=coordinate_system)
    if allow_legacy and isinstance(data,list):
        data=dict(found=True,bbox=data,coordinate_system=coordinate_system)
    if not isinstance(data,dict) or type(data.get("found")) is not bool: return None
    if not data["found"]: return None
    if data.get("coordinate_system")!=coordinate_system: return None
    box=data.get("bbox")
    if not isinstance(box,list) or len(box)!=4 or not all(finite(x) for x in box): return None
    x1,y1,x2,y2=box
    maximum=(width,height,width,height) if coordinate_system=="pixel" else (1000,)*4
    if any(v<0 or v>bound for v,bound in zip(box,maximum)) or x2<=x1 or y2<=y1: return None
    if coordinate_system=="normalized_1000":
        x1,x2=x1*width/1000,x2*width/1000
        y1,y2=y1*height/1000,y2*height/1000
    return (math.floor(x1),math.floor(y1),math.ceil(x2),math.ceil(y2))

def normalize_depth(depth,encoding):
    import numpy as np
    if encoding not in ("32FC1","16UC1"): raise ValueError("unsupported depth encoding")
    array=np.asarray(depth)
    if array.ndim!=2: raise ValueError("depth must be a two-dimensional aligned image")
    values=array.astype(np.float32,copy=True)
    if encoding=="16UC1": values*=.001
    values[~np.isfinite(values)|(values<=0)]=np.nan
    return values

def validate_snapshot(rgb_stamp,depth_stamp,now,rgb_shape,depth_shape,rgb_frame,depth_frame,max_skew=.05,max_age=1.):
    if not all(finite(v) for v in (rgb_stamp,depth_stamp,now,max_skew,max_age)) or min(max_skew,max_age)<0:
        raise ValueError("invalid snapshot clock")
    if rgb_stamp<0 or depth_stamp<0 or max(rgb_stamp,depth_stamp)>now+.05:
        raise ValueError("future/invalid observation")
    if now-min(rgb_stamp,depth_stamp)>max_age: raise ValueError("stale image")
    if abs(rgb_stamp-depth_stamp)>max_skew: raise ValueError("unsynchronized images")
    if tuple(rgb_shape[:2])!=tuple(depth_shape[:2]): raise ValueError("unaligned image sizes")
    if not rgb_frame or rgb_frame!=depth_frame: raise ValueError("image frame mismatch")

def validate_intrinsics(K):
    if len(K)!=9 or not all(finite(float(v)) for v in K) or K[0]<=0 or K[4]<=0:
        raise ValueError("invalid camera intrinsics")
