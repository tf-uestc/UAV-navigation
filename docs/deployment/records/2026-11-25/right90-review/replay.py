"""Recheck exact historical RGB frames with logged native replies; no API calls."""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image

here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p/'tests/offline/_support.py').is_file())
sys.path.insert(0, str(root/'tests/offline'))
from _support import load_detector

model = load_detector('qwen_vl_grounding')
detector = model.QwenVLGroundingDetector('offline-only')
source = json.loads((here/'source.json').read_text(encoding='utf-8-sig'))
report = {}
for trial, frames in source.items():
    report[trial] = []
    for frame in frames:
        rgb = np.array(Image.open(here/frame['file']).convert('RGB'))
        height, width = rgb.shape[:2]
        reply = json.dumps(frame['reply'])
        detector._call_api = lambda *args: reply
        box = detector.parse_response_bbox(reply, width, height)
        r,g,b = rgb.transpose(2,0,1)
        red = (r>110)&(r>1.5*g)&(r>1.5*b)
        blue = (b>110)&(b>1.5*g)&(b>1.5*r)
        row = dict(request=frame['id'], capture_stamp=frame['stamp'], image=frame['file'],
                   yaw_deg=math.degrees(frame['yaw']), scan_deg=frame['scan'],
                   phase=frame['heading'], box=box, red_pixels=int(red.sum()),
                   blue_pixels=int(blue.sum()), original_reason=frame['reason'],
                   original_status=frame['status'], original_map_point=frame['point'])
        if box:
            x1,y1,x2,y2 = box
            row['boxed_red_pixels'] = int(red[y1:y2,x1:x2].sum())
            row['boxed_blue_pixels'] = int(blue[y1:y2,x1:x2].sum())
        # The flight archive contains RGB, but no paired depth. Rejection
        # before depth fusion must therefore be invariant to the supplied depth.
        outcomes = []
        for depth_value in (np.nan, 2., 6.):
            depth = np.full((height,width),depth_value,np.float32)
            try:
                detector.detect(rgb,depth,'红色竖直杆',SimpleNamespace(K=(300,0,320,0,300,240,0,0,1)))
                outcome = 'SUCCEEDED'
            except model.DetectionRejected as exc:
                outcome = str(exc)
            outcomes.append(outcome)
        expected = 'target_color_unconfirmed' if box else 'target_not_found'
        assert outcomes == [expected]*3,(frame,outcomes)
        row['rgb_replay_reason'] = expected
        report[trial].append(row)
(here/'replay-results.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
for name, rows in report.items():
    for row in rows:
        print(name,row['request'],row['red_pixels'],row['blue_pixels'],row.get('boxed_blue_pixels'),
              row['original_status'],row['original_reason'],row['rgb_replay_reason'])
