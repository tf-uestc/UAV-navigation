"""Deterministic request lifetimes and bounded waits using injected monotonic time."""
from dataclasses import dataclass
from .contracts import RequestKey, finite

@dataclass(frozen=True)
class PendingRequest:
    key: RequestKey
    target: str
    deadline: float

class RequestRegistry:
    def __init__(self):
        self.mission_id = None
        self.next_id = 1
        self.pending = None
        self.reason = "idle"
    def begin(self, mission_id):
        RequestKey(mission_id,1)
        self.mission_id,self.pending,self.reason = mission_id,None,"ready"
    def cancel(self):
        self.mission_id,self.pending,self.reason = None,None,"cancelled"
    def issue(self,target,now,timeout):
        if self.mission_id is None or not target or not finite(now) or not finite(timeout) or timeout<=0:
            raise ValueError("invalid request")
        if self.pending is not None:
            raise ValueError("request already in flight")
        key=RequestKey(self.mission_id,self.next_id)
        self.next_id+=1
        self.pending=PendingRequest(key,target,now+timeout)
        self.reason="pending"
        return key
    def resolve(self,result,now,ros_now=None,max_age=None):
        if not finite(now): raise ValueError("invalid request clock")
        p=self.pending
        if p is None or result.key != p.key:
            self.reason="unsolicited"
            return False
        if result.target_text != p.target:
            self.reason="wrong_target"
            return False
        if now>=p.deadline:
            self.pending=None; self.reason="timeout"
            return False
        if ros_now is not None and result.status=="SUCCEEDED":
            if not finite(ros_now) or result.capture_stamp>ros_now+.05:
                self.reason="future_observation"; return False
            if max_age is not None and (not finite(max_age) or max_age<0 or ros_now-result.capture_stamp>max_age):
                self.reason="stale_observation"; return False
        self.pending=None
        self.reason=result.status.lower()
        return True
    def expire(self,now):
        if not finite(now): raise ValueError("invalid request clock")
        if self.pending is not None and now>=self.pending.deadline:
            key=self.pending.key
            self.pending=None; self.reason="timeout"
            return key
        return None

class WaitBudget:
    def __init__(self,retries=2):
        if type(retries) is not int or retries<0: raise ValueError("invalid retry budget")
        self.maximum=retries
        self.active=False
        self.retries=0
        self.token=None
        self.reason="idle"
    def enter(self,token,now,timeout):
        if not finite(now) or not finite(timeout) or timeout<=0: raise ValueError("invalid wait")
        self.token,self.timeout,self.deadline=token,timeout,now+timeout
        self.retries=0; self.active=True; self.reason="waiting"
    def complete(self,token):
        if self.active and token==self.token:
            self.active=False; self.reason="completed"; return True
        return False
    def cancel(self):
        self.active=False; self.reason="cancelled"
    def tick(self,now,ready=False):
        if not finite(now): raise ValueError("invalid wait clock")
        if not self.active: return "INACTIVE"
        if now<self.deadline: return "WAIT"
        if self.retries>=self.maximum:
            self.active=False; self.reason="timeout"; return "FAILED"
        self.retries+=1; self.deadline=now+self.timeout
        self.reason="retry" if ready else "not_ready"
        return "RETRY" if ready else "WAIT_READY"

class CallBudget:
    def __init__(self,max_calls=200,max_seconds=600):
        if type(max_calls) is not int or max_calls<1 or not finite(max_seconds) or max_seconds<=0:
            raise ValueError("invalid search budget")
        self.max_calls,self.max_seconds=max_calls,max_seconds
        self.calls=0; self.started=None
    def begin(self,now):
        if not finite(now): raise ValueError("invalid search clock")
        self.started,self.calls=now,0
    def reserve(self,now):
        if not finite(now) or self.started is None or now<self.started: return False
        if self.calls>=self.max_calls or now-self.started>=self.max_seconds: return False
        self.calls+=1
        return True
