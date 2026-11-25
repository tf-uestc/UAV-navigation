import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

from _support import ROOT

sys.path.insert(0,str(ROOT/'tools/deployment'))
from inspect_camera_height import audit, interpolate, table


class CameraHeightAuditTests(unittest.TestCase):
    def test_preserves_data_gap(self):
        data = table([dict(stamp=1.,z=1.),dict(stamp=2.,z=3.)], 'z')
        self.assertIsNone(interpolate(data,1.5,.2))
        self.assertEqual(interpolate(data,1.5,1.),2.)
        self.assertIsNone(interpolate(data,3.,1.))

    def test_separates_body_and_extrinsic_vertical_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder/'result').mkdir()
            trace=[]
            truth=[]
            for i in range(25):
                stamp = i/10
                airborne = stamp>=1.
                z = 1.75 if airborne else .1
                trace.append([stamp,'odom',dict(stamp=stamp,position=[0.,0.,z-(.2 if airborne else 0.)])])
                truth.append(dict(stamp=stamp,world_xyz=[0.,0.,z],world_speed_mps=0.))
            trace.append([.9,'fcu',dict(observed_sim=.9,armed=True)])
            sample=dict(stamp=1.5,speed_mps=.3,truth_time_offset_s=.05,truth_camera_z_m=1.75,
                        estimated_world_camera_z_m=1.45,world_camera_yaw_deg=0.,camera_to_target_horizontal_m=2.5)
            for path,data in [
                    ('clearance.json',dict(samples=truth)),
                    ('moving.json',dict(samples=[sample],ground_alignment_transform=np.eye(4).tolist(),
                                        ground_alignment_samples=20)),
                    ('result/trace.json',trace),
                    ('result/results.json',dict(checks=[dict(name='observed_and_held',start_sim=1.2,end_sim=2.)]))]:
                (folder/path).write_text(json.dumps(data))
            result=audit(folder)
            s=result['frames'][0]
            self.assertAlmostEqual(s['camera_residual_m'],-.3)
            self.assertAlmostEqual(s['body_height_residual_m'],-.2)
            self.assertAlmostEqual(s['camera_minus_body_residual_m'],-.1)
            self.assertAlmostEqual(s['tilt_only_effect_m'],0.)
            self.assertAlmostEqual(result['hold_body_height_residual_m']['median'],-.2)
