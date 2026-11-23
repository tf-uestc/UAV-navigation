import json,types,unittest
import numpy as np
from _support import load_detector

class VlmProtocolTests(unittest.TestCase):
    def make(self):
        m=load_detector('qwen_vl_grounding')
        m.requests.exceptions.ConnectTimeout=type('ConnectTimeout',(m.requests.exceptions.Timeout,),{})
        m.cv2.IMWRITE_JPEG_QUALITY=1
        m.cv2.imencode=lambda *a:(True,np.array([1,2,3],dtype=np.uint8))
        detector=m.QwenVLGroundingDetector(api_key='unit-test',timeout=35.)
        return m,detector
    def response(self,text):
        return types.SimpleNamespace(raise_for_status=lambda:None,
            json=lambda:{'choices':[{'message':{'content':text}}]})
    def test_protocol_is_explicit_system_instruction_and_image_remains_user_data(self):
        m,d=self.make();calls=[]
        def send(url,**kwargs):
            calls.append(kwargs);return self.response('{"found":false}')
        d._session.post=send
        self.assertEqual(d._call_api(np.zeros((10,20,3),np.uint8),'red pole'),'{"found":false}')
        payload=calls[0]['json']
        self.assertEqual([p['role'] for p in payload['messages']],['system','user'])
        self.assertIn('normalized_1000',payload['messages'][0]['content'])
        self.assertIn('color',payload['messages'][0]['content'])
        self.assertEqual(payload['messages'][1]['content'][1]['type'],'image_url')
        self.assertEqual(calls[0]['timeout'],(3.,35.))
    def test_empty_content_is_api_failure_not_target_absence(self):
        m,d=self.make();d._session.post=lambda *a,**k:self.response('')
        with self.assertRaises(m.DetectionRejected) as e:d._call_api(np.zeros((10,20,3),np.uint8),'red')
        self.assertEqual(str(e.exception),'invalid_api_response')
    def test_connection_read_and_http_failures_stay_distinct(self):
        m,d=self.make()
        timeout=m.requests.exceptions.Timeout
        m.requests.exceptions.ConnectTimeout=type('ConnectTimeout',(timeout,),{})
        for error,reason in [(m.requests.exceptions.ConnectTimeout(),'api_connect_timeout'),
                             (timeout(),'api_timeout')]:
            def send(*a,**k):raise error
            d._session.post=send
            with self.assertRaises(m.DetectionRejected) as e:d._call_api(np.zeros((10,20,3),np.uint8),'red')
            self.assertEqual(str(e.exception),reason)
        error=m.requests.exceptions.RequestException()
        error.response=types.SimpleNamespace(status_code=429)
        def send(*a,**k):raise error
        d._session.post=send
        with self.assertRaises(m.DetectionRejected) as e:d._call_api(np.zeros((10,20,3),np.uint8),'red')
        self.assertEqual(str(e.exception),'api_http_429')

    def test_format_feedback_changes_hint_but_does_not_retry_or_relax_parser(self):
        m,d=self.make();calls=[]
        def send(url,**kwargs):
            calls.append(kwargs);return self.response('{"found":false}')
        d._session.post=send
        d._call_api(np.zeros((10,20,3),np.uint8),'red','invalid_bbox_response')
        self.assertEqual(len(calls),1)
        text=calls[0]['json']['messages'][0]['content']
        self.assertIn('previous reply failed',text)
        self.assertIn('bbox_2d',text)
        self.assertIsNone(d._parse_bbox('{"bbox_2d":[1,2,3,4]}',20,10,'normalized_1000'))


    def test_native_grounding_has_explicit_units_at_multiple_image_sizes(self):
        m,d=self.make()
        for w,h in ((320,240),(640,480),(1280,960)):
            for key in ("bbox","bbox_2d"):
                payload={"label":"red pole",key:[250,250,750,750]}
                for raw in (payload,[payload]):
                    self.assertEqual(d.parse_response_bbox(json.dumps(raw),w,h),
                                     (w//4,h//4,3*w//4,3*h//4))
                    self.assertIsNone(d._parse_bbox(json.dumps(raw),w,h,"normalized_1000"))

    def test_native_rejects_ambiguous_or_conflicting_envelopes(self):
        m,d=self.make()
        payloads=[{"bbox_2d":[1,2,3,4]},
                  {"label":"","bbox_2d":[1,2,3,4]},
                  {"label":"red","bbox":[1,2,3,4],"bbox_2d":[1,2,3,4]},
                  {"label":"red","bbox":[1,2,3,4],"coordinate_system":"pixel"},
                  {"found":True,"bbox":[1,2,3,4]},
                  [1,2,3,4],
                  [{"label":"red","bbox":[1,2,3,4]}]*2,
                  {"label":"red","bbox_2d":[2,2,1,4]},
                  {"label":"red","bbox_2d":[-1,2,3,4]},
                  {"label":"red","bbox_2d":[1,2,1001,4]},
                  {"label":"red","bbox_2d":[True,2,3,4]},
                  {"label":"red","bbox_2d":[1,2,float("nan"),4]}]
        for payload in payloads:
            self.assertIsNone(d.parse_response_bbox(json.dumps(payload),640,480),payload)

    def test_native_contract_never_applies_to_another_model_or_pixel_units(self):
        m,d=self.make()
        for kwargs in (dict(model="other"),dict(coordinate_system="pixel")):
            with self.assertRaises(ValueError):
                m.QwenVLGroundingDetector("unit-test",response_protocol="qwen_native_1000",**kwargs)
        other=m.QwenVLGroundingDetector("unit-test",model="other")
        self.assertEqual(other.response_protocol,"canonical")
        self.assertIsNone(other.parse_response_bbox('{"label":"red","bbox_2d":[1,2,3,4]}',640,480))

    def test_native_and_canonical_absence_and_explicit_metadata_remain_distinct(self):
        m,d=self.make()
        self.assertIsNone(d.parse_response_bbox('{"found":false}',640,480))
        self.assertIsNone(d.parse_response_bbox('{"found":true,"bbox":[1,2,3,4],"coordinate_system":"pixel"}',640,480))
        self.assertEqual(d.parse_response_bbox('{"found":true,"bbox":[250,250,750,750],"coordinate_system":"normalized_1000"}',640,480),(160,120,480,360))


    def test_native_optional_found_and_matching_units_are_compatible(self):
        m,d=self.make()
        for name in ("bbox","bbox_2d"):
            base={"label":"red pole",name:[250,250,750,750]}
            for extra in ({"found":True},{"coordinate_system":"normalized_1000"},
                          {"found":True,"coordinate_system":"normalized_1000"}):
                for payload in (dict(base,**extra),[dict(base,**extra)]):
                    self.assertEqual(d.parse_response_bbox(json.dumps(payload),640,480),(160,120,480,360))

    def test_contradictory_absence_and_duplicate_fields_are_format_errors(self):
        m,d=self.make()
        rgb=np.zeros((24,32,3),np.uint8);depth=np.full((24,32),3.,np.float32)
        info=types.SimpleNamespace(K=[100,0,16,0,100,12,0,0,1])
        replies=['{"found":false,"label":"red","bbox_2d":[100,100,500,500]}',
                 '{"found":false,"bbox":[100,100,500,500],"coordinate_system":"normalized_1000"}',
                 '{"found":true,"found":false}',
                 '{"label":"red","bbox_2d":[1,2,3,4],"bbox_2d":[5,6,7,8]}',
                 '{"found":false,"coordinate_system":"pixel"}']
        for raw in replies:
            d._call_api=lambda *a,**k:raw
            with self.assertRaises(m.DetectionRejected) as e:d.detect(rgb,depth,"red pole",info)
            self.assertEqual(str(e.exception),"invalid_bbox_response",raw)

    def test_native_empty_list_is_explicit_no_detection_only_for_native_protocol(self):
        m,d=self.make()
        rgb=np.zeros((24,32,3),np.uint8);depth=np.full((24,32),3.,np.float32)
        info=types.SimpleNamespace(K=[100,0,16,0,100,12,0,0,1])
        d._call_api=lambda *a,**k:"[]"
        with self.assertRaises(m.DetectionRejected) as e:d.detect(rgb,depth,"red",info)
        self.assertEqual(str(e.exception),"target_not_found")
        canonical=m.QwenVLGroundingDetector("unit-test",response_protocol="canonical")
        canonical._call_api=d._call_api
        with self.assertRaises(m.DetectionRejected) as e:canonical.detect(rgb,depth,"red",info)
        self.assertEqual(str(e.exception),"invalid_bbox_response")

    def test_nonfinite_and_invalid_found_never_become_absence(self):
        m,d=self.make()
        rgb=np.zeros((24,32,3),np.uint8);depth=np.full((24,32),3.,np.float32)
        info=types.SimpleNamespace(K=[100,0,16,0,100,12,0,0,1])
        for raw in ('{"found":0}','{"found":"false"}','{"found":null}',
                    '{"found":false,"bbox":NaN}','{"found":false,"unknown":1}'):
            d._call_api=lambda *a,**k:raw
            with self.assertRaises(m.DetectionRejected) as e:d.detect(rgb,depth,"red",info)
            self.assertEqual(str(e.exception),"invalid_bbox_response",raw)

    def test_complete_native_envelope_still_uses_depth_and_camera_contract(self):
        m,d=self.make()
        d._call_api=lambda *a,**k:'{"found":true,"label":"red pole","bbox_2d":[250,250,750,750]}'
        rgb=np.zeros((24,32,3),np.uint8);depth=np.full((24,32),3.,np.float32)
        info=types.SimpleNamespace(K=[100,0,16,0,100,12,0,0,1])
        found=d.detect(rgb,depth,"red",info)
        self.assertEqual(found.bbox,(8,6,24,18))
        self.assertAlmostEqual(found.depth_m,3.)
        self.assertEqual(found.point_camera,(-.015,-.015,3.))

    def test_truncated_api_completion_is_invalid_api_response(self):
        m,d=self.make()
        d._session.post=lambda *a,**k:types.SimpleNamespace(raise_for_status=lambda:None,
            json=lambda:{"choices":[{"finish_reason":"length","message":{"content":"{}"}}]})
        with self.assertRaises(m.DetectionRejected) as e:d._call_api(np.zeros((10,20,3),np.uint8),"red")
        self.assertEqual(str(e.exception),"invalid_api_response")


    def test_explicit_legacy_opt_in_is_preserved_without_weakening_default(self):
        m,d=self.make()
        old=m.QwenVLGroundingDetector("unit-test",model="other",coordinate_system="pixel",allow_legacy_bbox=True)
        for text in ("[1,2,3,4]","<box>[1,2,3,4]</box>"):
            self.assertEqual(old.parse_response_bbox(text,20,10),(1,2,3,4))
            self.assertIsNone(d.parse_response_bbox(text,20,10))
        self.assertIsNone(old.parse_response_bbox('{"found":true,"found":true,"bbox":[1,2,3,4],"coordinate_system":"pixel"}',20,10))

    def test_fences_and_supported_absence_shapes_do_not_require_a_guessed_box(self):
        m,d=self.make()
        fence=chr(96)*3
        positive='{"found":true,"label":"red","bbox_2d":[250,250,750,750]}'
        self.assertEqual(d.parse_response_bbox(fence+"JSON\n"+positive+"\n"+fence,640,480),(160,120,480,360))
        self.assertIsNone(d.parse_response_bbox("Here is the box: "+positive,640,480))
        self.assertIsNone(d.parse_response_bbox(json.dumps([json.loads(positive),json.loads(positive)]),640,480))
        for raw in ('{"found":false}','[{"found":false}]','{"found":false,"bbox":null}','{"found":false,"bbox_2d":[]}'):
            with self.assertRaises(m.DetectionRejected) as e:d.parse_response(raw,640,480)
            self.assertEqual(str(e.exception),"target_not_found")
