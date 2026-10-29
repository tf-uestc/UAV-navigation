import unittest
from _support import ROOT
from navigation_core.lifecycle import WaitBudget,CallBudget
class WaitTests(unittest.TestCase):
    def test_bounded_not_ready(self):
        w=WaitBudget(2); w.enter("goal",0,10)
        self.assertEqual(w.tick(9),"WAIT")
        self.assertEqual(w.tick(10),"WAIT_READY")
        self.assertEqual(w.tick(20),"WAIT_READY")
        self.assertEqual(w.tick(30),"FAILED")
        self.assertEqual(w.tick(1000),"INACTIVE")
    def test_ready_retry_and_matching_completion(self):
        w=WaitBudget(); w.enter("new",0,10)
        self.assertEqual(w.tick(10,True),"RETRY")
        self.assertFalse(w.complete("old"))
        self.assertTrue(w.complete("new"))
        self.assertEqual(w.tick(100),"INACTIVE")
    def test_deadline_exact_cancel(self):
        w=WaitBudget(0); w.enter(1,0,20)
        self.assertEqual(w.tick(19.999),"WAIT"); self.assertEqual(w.tick(20),"FAILED")
        w.enter(2,21,20); w.cancel(); self.assertEqual(w.tick(100),"INACTIVE")
    def test_call_budget_time_and_count(self):
        b=CallBudget(2,30); b.begin(0)
        self.assertTrue(b.reserve(0)); self.assertTrue(b.reserve(1)); self.assertFalse(b.reserve(2))
        b.begin(0); self.assertFalse(b.reserve(30)); self.assertFalse(b.reserve(-1))
