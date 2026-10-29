import json
import unittest
import numpy as np
from _support import ROOT
from navigation_core.perception import parse_bbox,normalize_depth,validate_snapshot,validate_intrinsics
def response(box,system="pixel"):
    return json.dumps(dict(found=True,bbox=box,coordinate_system=system))
class PerceptionTests(unittest.TestCase):
    def test_explicit_coordinate_systems(self):
        self.assertEqual(parse_bbox(response([100,100,200,200]),640,480),(100,100,200,200))
        self.assertEqual(parse_bbox(response([100,100,200,200],"normalized_1000"),640,480,"normalized_1000"),(64,48,128,96))
        self.assertIsNone(parse_bbox(response([100,100,200,200],"normalized_1000"),640,480))
    def test_missing_multibox_not_found_invalid(self):
        for raw in ['{"found":false}',"[100,100,200,200]",
                    response([[1,2,3,4],[4,5,6,7]]),response([1,1,1,2]),
                    response([-1,1,2,2]),response([1,1,float("nan"),2])]:
            self.assertIsNone(parse_bbox(raw,640,480))
        self.assertEqual(parse_bbox("[100,100,200,200]",640,480,allow_legacy=True),(100,100,200,200))
        self.assertEqual(parse_bbox("```json\n"+response([1,2,3,4])+"\n```",640,480),(1,2,3,4))
    def test_depth_units_invalid_and_input_immutable(self):
        raw=np.array([[1000,0],[2000,3000]],dtype=np.uint16); original=raw.copy()
        values=normalize_depth(raw,"16UC1")
        self.assertAlmostEqual(float(values[0,0]),1,places=6)
        self.assertTrue(np.isnan(values[0,1])); np.testing.assert_array_equal(raw,original)
        np.testing.assert_allclose(values[1],normalize_depth([[2,3]],"32FC1")[0])
        self.assertTrue(np.isnan(normalize_depth([[float("inf"),float("nan"),-1]],"32FC1")).all())
        with self.assertRaises(ValueError): normalize_depth([[1]],"bad")
        with self.assertRaises(ValueError): normalize_depth([[[1]]],"32FC1")
    def test_snapshot_and_calibration(self):
        validate_snapshot(10,10.01,10.1,(480,640,3),(480,640),"cam","cam")
        cases=[(10,10.2,10.3,(480,640,3),(480,640),"cam","cam"),
               (1,1,10,(480,640,3),(480,640),"cam","cam"),
               (10,10,10,(480,640,3),(240,320),"cam","cam"),
               (10,10,10,(480,640,3),(480,640),"cam","other")]
        for args in cases:
            with self.assertRaises(ValueError): validate_snapshot(*args)
        validate_intrinsics([500,0,320,0,500,240,0,0,1])
        with self.assertRaises(ValueError): validate_intrinsics([0]*9)
