import json
from pathlib import Path
import sys
import tempfile
import unittest
import zlib
import struct

import numpy as np

from _support import ROOT
sys.path.insert(0,str(ROOT/'tools/deployment'))
from summarize_visual_attempts import report


def write_json(path, data):
    path.write_text(json.dumps(data),encoding='utf-8')


class ArchiveReviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.label='V03-2.4-front-1'
        self.case=self.root/'flights'/self.label
        for part in ('result','images'):
            (self.case/part).mkdir(parents=True)
        self.mid='red-1'
        cap=dict(mission_id=self.mid,request_id=1,stop_confirmed=True,floor=5.)
        reply=dict(mission_id=self.mid,request_id=1,capture_stamp=5.,completed_stamp=6.,status='SUCCEEDED',reason='')
        self.trace=[[1.,'fcu',dict(connected=True,armed=True)],
                    [4.9,'mission',dict(mission_id=self.mid,phase='WAIT_VLM',detection_request_id=1)],
                    [5.,'search_capture',cap],[6.,'detect_result',reply],[6.,'mission',
                    dict(mission_id=self.mid,phase='DONE',reason='observation_standoff_reached',observed_sim=6.)]]
        points=[]
        for i in range(106):
            t=6.+i/10.
            self.trace.extend([[t,'flight',dict(observed_sim=t,mode='HOLD',context=None,owns_flight=True,yaw_reference=0.)],
                               [t,'odom',dict(stamp=t,yaw=0.)]])
            points.append(dict(stamp=t,world_xyz=[0.,0.,1.5],target_xyz=[2.4,0.,.5],
                               world_speed_mps=0.,world_yaw=0.))
        self.trace.extend([[17.,'flight_command',dict(command='LAND',observed_sim=17.)],
                           [19.,'fcu',dict(connected=True,armed=False)]])
        self.results=dict(passed=True,final_armed=False,checks=[
            dict(name='attempt_observations',mission_id=self.mid,terminal='DONE',segments=1,
                 distance_at_terminal_m=2.4,reason='observation_standoff_reached',scans=[]),
            dict(name='landed_disarmed'),
            dict(name='observed_and_held',start_sim=6.,end_sim=16.5),
            dict(name='historical_tf_gazebo_and_target_surface',samples=[
                dict(request_id=1,capture_stamp=5.,truth_time_delta_s=.01,target_surface_error_m=.02)])])
        write_json(self.case/'clearance.json',dict(samples=points,vehicle_bounding_radius_m=.35))
        write_json(self.case/'physics-contacts.json',dict(passed=True,physics_messages=500,
                                                         start_stamp=1.,last_message_stamp=19.))
        (self.case/'physics-contacts.txt').write_text('collision1: "contact_witness::a"\ncollision2: "ground_plane::b"')
        (self.case/'commit.txt').write_text('abcd')
        (self.case/'flight.exit').write_text('0')
        moving=[dict(truth_camera_z_m=1.0+i*.01,estimated_world_camera_z_m=1.02+i*.01,
                     delayed_tf_max_difference=0.,gazebo_surface_error_p95_m=.01,
                     map_surface_error_p95_m=.07,camera_to_target_horizontal_m=2.4) for i in range(10)]
        write_json(self.case/'moving.json',dict(passed=True,samples=moving))
        write_json(self.case/'acceptance.json',dict(passed=True))
        (self.case/'launch.log').write_text('Querying DashScope (model)...\n')
        def chunk(kind, data):
            return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data))
        png=(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>2I5B',1,1,8,2,0,0,0))+
             chunk(b'IDAT',zlib.compress(b'\0\x80\x80\x80'))+chunk(b'IEND',b''))
        self.frame=self.case/'images'/'image.png'
        self.frame.write_bytes(png)
        depth='red-1-1-5000000000-depth.npz'
        np.savez_compressed(self.case/'images'/depth,depth=np.ones((1,1)))
        write_json(self.case/'images/observations.json',dict(rows=[dict(request=cap,result=reply,exact_frame='image.png',
                   exact_depth=depth,frames=[dict(stamp=5.,file='image.png')])]))
        self.ledger=self.root/'V03-attempts.jsonl'
        self.ledger.write_text(json.dumps(dict(event='start',label=self.label))+'\n'+
                               json.dumps(dict(event='finish',label=self.label,exit_code=0))+'\n')

    def check_report(self):
        write_json(self.case/'result/trace.json',self.trace)
        write_json(self.case/'result/results.json',self.results)
        return report([self.root],['V*'])

    def test_finished_success_and_started_unfinished_both_in_denominator(self):
        with self.ledger.open('a') as f:
            f.write(json.dumps(dict(event='start',label='V03-2.4-front-2'))+'\n')
        r=self.check_report()
        self.assertEqual(r['overall']['started'],2)
        self.assertEqual(r['by_direction']['2.4/front']['success_rate'],.5,r)
        self.assertEqual(r['attempts'][0]['hold_seconds'],10.5)
        self.assertEqual(r['attempts'][0]['cloud_calls'],1)

    def test_rejects_incorrect_auditor_pass_even_if_latest_image_or_localization_missing(self):
        self.frame.unlink()
        self.results['checks'][-1]['samples'][0]['request_id']=3
        row=self.check_report()['attempts'][0]
        self.assertTrue(row['auditor_passed'])
        self.assertFalse(row['individually_verified'])
        self.assertIn('exact_observations',row['failed_checks'])
        self.assertIn('latest_reply',row['failed_checks'])

    def test_rejects_early_landing_and_short_truth_even_if_checker_passed(self):
        self.trace[-2][2]['observed_sim']=15.
        self.results['checks'][-2]['end_sim']=15.8
        row=self.check_report()['attempts'][0]
        self.assertIn('pre_land_hold',row['failed_checks'])
        self.assertIn('hold_truth',row['failed_checks'])

    def test_catches_unexpected_contact_in_raw_log(self):
        with (self.case/'physics-contacts.txt').open('a') as f:
            f.write('\ncollision1: "iris_depth_camera::base"\ncollision2: "wall::box"\n')
        row=self.check_report()['attempts'][0]
        self.assertEqual(row['unexpected_contacts'],['wall::box'])
        self.assertIn('contact_evidence',row['failed_checks'])

    def test_detects_missing_ground_probe_and_bad_moving_camera_samples(self):
        (self.case/'flight.exit').unlink()
        moving=json.loads((self.case/'moving.json').read_text())
        moving['samples'][-1]['gazebo_surface_error_p95_m']=.25
        write_json(self.case/'moving.json',moving)
        row=self.check_report()['attempts'][0]
        self.assertIn('landing',row['failed_checks'])
        self.assertIn('moving_camera',row['failed_checks'])
        self.assertAlmostEqual(row['camera_abs_z_error_max_m'],.02)
        self.assertTrue(row['audit_disagreement'])

    def test_reports_unfinished_trailing_ledger_record(self):
        with self.ledger.open('a') as f:
            f.write('{"event":')
        data=self.check_report()
        self.assertEqual(len(data['ledger_warnings']),1)
        self.assertEqual(data['by_version_total']['abcd']['passed'],1,data)

    def test_shared_archive_handles_planning_batch_metadata(self):
        ledger=self.root/'F04-attempts.jsonl'
        ledger.write_text('{"event":"batch"}\n{"event":"complete"}\n',encoding='utf-8')
        self.check_report()
        self.assertEqual(report([self.root],['*'])['by_version_total']['abcd']['passed'],1)

    def test_false_negative_in_primary_audit_is_visible(self):
        write_json(self.case/'acceptance.json',dict(passed=False,errors=['unexpected']))
        data=self.check_report()
        row=data['attempts'][0]
        self.assertTrue(row['individually_verified'],row)
        self.assertTrue(row['audit_disagreement'])
        self.assertEqual(data['overall']['passed'],0)
