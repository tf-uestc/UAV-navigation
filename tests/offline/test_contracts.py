import unittest
from _support import ROOT
from navigation_core.contracts import RequestKey,DetectionResult
class ContractTests(unittest.TestCase):
    def test_roundtrip(self):
        r=DetectionResult(RequestKey("a",901),"red","SUCCEEDED",1,2,(1,2,3))
        self.assertEqual(DetectionResult.from_json(r.to_json()),r)
    def test_invalid(self):
        for point in [(1,float("nan"),3),(1,2),None]:
            with self.assertRaises(ValueError): DetectionResult(RequestKey("a",1),"red","SUCCEEDED",1,2,point)
        with self.assertRaises(ValueError): DetectionResult(RequestKey("a",1),"red","FAILED",2,1)
        with self.assertRaises(ValueError): RequestKey("a",True)
        with self.assertRaises(ValueError): RequestKey("",1)
    def test_mission_scopes_ids(self):
        self.assertNotEqual(RequestKey("a",1),RequestKey("b",1))
