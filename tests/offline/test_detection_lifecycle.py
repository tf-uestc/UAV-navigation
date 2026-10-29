import unittest
from _support import ROOT
from navigation_core.contracts import RequestKey,DetectionResult
from navigation_core.lifecycle import RequestRegistry
def result(key,status="SUCCEEDED",target="red",capture=1):
    return DetectionResult(key,target,status,capture,2,(1,2,3) if status=="SUCCEEDED" else None)
class LifecycleTests(unittest.TestCase):
    def test_old_task_and_duplicate(self):
        r=RequestRegistry(); r.begin("a"); old=r.issue("red",0,20)
        r.begin("b"); new=r.issue("red",1,20)
        self.assertFalse(r.resolve(result(old),2)); self.assertIsNotNone(r.pending)
        self.assertTrue(r.resolve(result(new),2)); self.assertFalse(r.resolve(result(new),3))
    def test_busy_and_failed_are_terminal(self):
        for status in ("BUSY","FAILED"):
            r=RequestRegistry(); r.begin("a"); key=r.issue("red",0,20)
            self.assertTrue(r.resolve(result(key,status),1)); self.assertIsNone(r.pending)
            self.assertFalse(r.resolve(result(key),2))
    def test_timeout_once_and_cancel(self):
        r=RequestRegistry(); r.begin("a"); key=r.issue("red",0,20)
        self.assertIsNone(r.expire(19.9)); self.assertEqual(r.expire(20),key)
        self.assertIsNone(r.expire(21)); self.assertFalse(r.resolve(result(key),21))
        key=r.issue("red",22,20); r.cancel(); self.assertFalse(r.resolve(result(key),23))
    def test_wrong_target_age_and_future_do_not_consume(self):
        r=RequestRegistry(); r.begin("a"); key=r.issue("red",0,20)
        self.assertFalse(r.resolve(result(key,target="blue"),2))
        self.assertFalse(r.resolve(result(key),2,ros_now=100,max_age=30))
        self.assertFalse(r.resolve(result(key,capture=2),2,ros_now=1,max_age=30))
        self.assertTrue(r.resolve(result(key),2,ros_now=2,max_age=30))
    def test_immediate_callback_after_registration(self):
        r=RequestRegistry(); r.begin("a"); key=r.issue("red",0,20)
        self.assertTrue(r.resolve(result(key,"BUSY"),.001))
