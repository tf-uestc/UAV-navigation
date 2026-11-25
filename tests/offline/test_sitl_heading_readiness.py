"""Test the optional PX4 SITL takeoff heading gate and its local telemetry source."""
import importlib.util
import json
import sys
import time
import types
import unittest
from unittest.mock import patch

from _support import ros_modules, String, Time, SCRIPTS
from sitl_heading_readiness import HeadingSamples, parse_position

class HeadingGuardTests(unittest.TestCase):
    def setUp(self):
        stubs=ros_modules()
        stubs['sensor_msgs'].point_cloud2=types.SimpleNamespace()
        stubs['sensor_msgs.msg'].PointCloud2=object
        loader=importlib.util.spec_from_file_location('candidate_mission',SCRIPTS/'segment_mission.py')
        module=importlib.util.module_from_spec(loader)
        with patch.dict(sys.modules,stubs): loader.loader.exec_module(module)
        self.node=module.SegmentMission()
        self.node.require_sitl_heading=True
        self.node.mode='visual'
        self.wall=time.monotonic()
        self.node.flight_status=lambda:dict(mode='HOLD',owns_flight=True,flight_session_id='f',ready=True)
        self.node.flight_session='f'
        self.node.phase='TAKEOFF'
        self.node.deadline=self.wall+2.
        self.node.odom=None
        self.old=Time.current
        self.addCleanup(setattr,Time,'current',self.old)
        Time.current=10.

    def test_telemetry_parser_matches_px4_print_and_rejects_unknown(self):
        result=parse_position('timestamp_sample: 1234000 (100 us before timestamp)\n heading_good_for_control: True\n heading_reset_counter: 2')
        self.assertAlmostEqual(result[0],1.234)
        self.assertEqual(result[1:],(True,2))
        with self.assertRaises(KeyError): parse_position('no data')

    def test_requires_continuous_fresh_good_samples_and_reset_restarts_wait(self):
        gate=HeadingSamples()
        for t in (3.,3.4,3.8): self.assertFalse(gate.update(t,True,1,t+.01))
        self.assertTrue(gate.update(4.,True,1,4.01))
        self.assertFalse(gate.update(4.2,True,2,4.21))
        self.assertFalse(gate.update(4.4,True,2,5.3))
        self.assertFalse(gate.update(5.4,True,2,5.4))
        self.assertFalse(gate.update(5.5,False,2,5.5))

    def test_ignores_missing_future_or_stale_telemetry(self):
        n=self.node
        self.assertFalse(n.initial_heading_ready(self.wall))
        for stamp,wall in [(8.,self.wall),(12.,self.wall),(10.,self.wall-2)]:
            n.heading_sample=(stamp,True,wall,10.)
            self.assertFalse(n.initial_heading_ready(self.wall))
        n.heading_sample=(10.,True,self.wall,10.)
        self.assertTrue(n.initial_heading_ready(self.wall))
        n.mode='manual';n.heading_sample=None
        self.assertTrue(n.initial_heading_ready(self.wall))

    def test_managed_takeoff_uses_visual_height_but_preserves_cruise_height(self):
        n=self.node;n.phase='IDLE';n.height=1.3;n.visual_takeoff_height=1.6
        n.flight_status=lambda:dict(mode='IDLE',owns_flight=False,flight_session_id='f',ready=True)
        n.feedback=lambda:((0.,0.,0.),(0.,0.,0.),0.,0.,10.,self.wall)
        sent=[]
        n.command_flight=lambda cmd,**kwargs:sent.append((cmd,kwargs))
        n.on_command(String(json.dumps(dict(command='START',mission_session_id=n.session,mission_id='probe',target='rod',takeoff=True))))
        self.assertEqual(sent,[('TAKEOFF',dict(height=1.6))])
        self.assertEqual(n.height,1.3)

    def message(self,stamp,ready=True,good=True,counter=2):
        self.node.on_heading_ready(String(json.dumps(dict(stamp=stamp,ready=ready,
                                                          heading_good=good,counter=counter))))

    def test_late_good_cannot_override_a_newer_fault_or_revive_same_stamp(self):
        self.message(9.8)
        self.message(9.95,False,False)
        self.message(9.85)
        self.message(9.95)
        self.assertEqual(self.node.heading_sample[:2],(9.95,False))
        self.message(10.)
        self.assertTrue(self.node.initial_heading_ready(time.monotonic()))
        self.message(10.,True,False)
        self.assertFalse(self.node.initial_heading_ready(time.monotonic()))

    def test_duplicate_cannot_refresh_wall_timestamp_or_promote_reset(self):
        self.message(10.)
        old=self.node.heading_sample
        self.message(10.)
        self.assertEqual(old,self.node.heading_sample)
        g=HeadingSamples()
        self.assertFalse(g.update(1.,True,3,1.))
        self.assertFalse(g.update(1.,True,3,1.4))
        self.assertFalse(g.update(1.,True,3,1.8))
        self.assertFalse(g.update(1.9,True,4,1.9))
        self.assertFalse(g.update(1.8,True,3,1.9))

    def test_invalid_health_and_counter_never_open_gate(self):
        for i,(good,counter) in enumerate([(False,3),(True,-1),(True,None),(True,True)]):
            stamp=9.7+i*.05
            self.message(stamp,True,good,counter)
            self.assertFalse(self.node.initial_heading_ready(time.monotonic()))

    def test_new_task_and_restart_discard_previous_valid_sample(self):
        n=self.node
        self.message(10.)
        Time.current=1.
        self.message(.95,False,False)
        self.assertEqual(n.heading_sample[:2],(.95,False))
        self.message(1.)
        self.assertTrue(n.initial_heading_ready(time.monotonic()))
        n.phase='IDLE';n.flight_status=lambda:dict(mode='IDLE',owns_flight=False,flight_session_id='f',ready=True)
        n.feedback=lambda:((0.,0.,0.),(0.,0.,0.),0.,0.,1.,self.wall)
        n.command_flight=lambda *_a,**_kw:None
        n.on_command(String(json.dumps(dict(command='START',mission_session_id=n.session,mission_id='after_reboot',
                                            target='red pole',takeoff=True))))
        self.assertEqual(n.phase,'TAKEOFF')
        self.assertIsNone(n.heading_sample)
        n.tick(self.wall+.3)
        self.assertEqual(n.phase,'TAKEOFF')

    def test_no_heading_sample_times_out_lands_and_never_requests_detection(self):
        n=self.node
        sent=[]
        n.command_flight=lambda cmd,**kwargs:sent.append(cmd)
        n.tick(self.wall+.4)
        self.assertEqual(n.phase,'TAKEOFF')
        n.tick(self.wall+2.1)
        self.assertEqual(sent,['LAND'])
        self.assertEqual((n.phase,n.reason),('FAILED','takeoff_heading_unavailable'))

    def test_valid_and_stable_heading_releases_into_original_stop_phase(self):
        self.message(10.)
        self.node.tick(time.monotonic())
        self.assertEqual(self.node.phase,'BRAKE')
        self.assertEqual(self.node.after_stop[0],'DETECT')


if __name__=='__main__':unittest.main(verbosity=2)
