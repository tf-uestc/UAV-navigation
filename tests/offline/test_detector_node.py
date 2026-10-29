import json
import threading
import types
import unittest
import numpy as np
from _support import load_detector,Time,Publisher,Header,Point,String
from navigation_core.contracts import DetectionResult
class DetectorNodeTests(unittest.TestCase):
    def make(self):
        m=load_detector(); n=m.DetectorNode.__new__(m.DetectorNode)
        n.lock=threading.Lock(); n._detect_lock=threading.Lock()
        n._detecting=True; n._active_request_id=1; n._active_mission_id="a"
        n._managed_only=True; n._max_skew=.05; n._max_image_age=1
        n.pub_target=Publisher("/target"); n.pub_status=Publisher("/status")
        n.pub_result=Publisher("/result"); n.pub_debug=Publisher("/debug")
        n.camera_optical_frame="optical"; n.target_frame="map"
        Time.current=100.1
        n.latest_rgb=types.SimpleNamespace(header=Header(stamp=Time(100),frame_id="cam"),array=np.zeros((4,4,3),dtype=np.uint8))
        n.latest_depth=types.SimpleNamespace(header=Header(stamp=Time(100),frame_id="cam"),array=np.full((4,4),1000,dtype=np.uint16),encoding="16UC1")
        n.latest_cam_info=types.SimpleNamespace(width=4,height=4,K=[2,0,2,0,2,2,0,0,1])
        n.bridge=m.CvBridge()
        detected=types.SimpleNamespace(point_camera=(0.,0.,3.))
        n.detector=types.SimpleNamespace(detect=lambda *a:detected)
        n._draw_debug=lambda *a:np.zeros((4,4,3),dtype=np.uint8)
        n.tf_buffer=types.SimpleNamespace(transform=lambda p,*a:types.SimpleNamespace(point=Point(10,0,0)))
        return m,n
    def test_result_keeps_business_id_when_legacy_seq_changes(self):
        m,n=self.make(); n._run_detection("red",1,"a")
        result=DetectionResult.from_json(n.pub_result.messages[0].data)
        self.assertEqual(result.key.mission_id,"a"); self.assertEqual(result.key.request_id,1)
        self.assertEqual(result.status,"SUCCEEDED")
        self.assertEqual(n.pub_target.messages[0].header.seq,0)
        self.assertFalse(n._detecting)
    def test_historical_tf_failure_never_uses_latest(self):
        m,n=self.make(); calls=[]
        def fail(point,*args):
            calls.append(point.header.stamp.to_sec())
            raise m.tf2_ros.ExtrapolationException()
        n.tf_buffer.transform=fail
        n._run_detection("red",1,"a")
        self.assertEqual(calls,[100])
        self.assertEqual(DetectionResult.from_json(n.pub_result.messages[0].data).status,"FAILED")
        self.assertFalse(n._detecting)
    def test_busy_identifies_incoming_not_active_request(self):
        m,n=self.make()
        n._cb_instruction(String(json.dumps({"target":{"text":"blue"},"_mission_mgr_trigger":True,"mission_id":"b","request_id":9})))
        result=DetectionResult.from_json(n.pub_result.messages[0].data)
        self.assertEqual((result.key.mission_id,result.key.request_id,result.target_text,result.status),("b",9,"blue","BUSY"))
        self.assertEqual(n._active_request_id,1)
    def test_bad_point_and_unsynced_frames_release_worker(self):
        for bad in ("point","sync"):
            m,n=self.make()
            if bad=="point": n.tf_buffer.transform=lambda *a:types.SimpleNamespace(point=Point(float("nan"),0,0))
            else: n.latest_depth.header.stamp=Time(99)
            n._run_detection("red",1,"a")
            result=DetectionResult.from_json(n.pub_result.messages[0].data)
            self.assertEqual(result.status,"FAILED"); self.assertIsNone(result.point)
            self.assertFalse(n._detecting)
    def test_model_parser_uses_declared_protocol(self):
        m=load_detector("qwen_vl_grounding")
        text=json.dumps({"found":True,"bbox":[100,100,200,200],"coordinate_system":"pixel"})
        self.assertEqual(m.QwenVLGroundingDetector._parse_bbox(text,640,480),(100,100,200,200))

    def test_busy_publication_releases_request_lock(self):
        m,n=self.make(); locks=[]
        def observe(message):
            acquired=n._detect_lock.acquire(blocking=False)
            locks.append(acquired)
            if acquired: n._detect_lock.release()
        n.pub_result.on_publish=observe
        n._cb_instruction(String(json.dumps({"target":{"text":"blue"},"_mission_mgr_trigger":True,"mission_id":"b","request_id":9})))
        self.assertEqual(locks,[True])
