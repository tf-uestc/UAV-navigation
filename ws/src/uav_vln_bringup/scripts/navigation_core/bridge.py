"""Nonblocking bridge decision core; not wired to a vehicle until Ubuntu U07."""
import math
from dataclasses import dataclass
from .contracts import finite,vector3
from .control import Reference
from .yaw import quaternion_yaw

@dataclass(frozen=True)
class Feedback:
    position: tuple
    yaw: float
    connected: bool = True
    armed: bool = False
    mode: str = "POSCTL"
    landed: bool = True
    pose_age: float = 0.
    speed: float = 0.
    def __post_init__(self):
        object.__setattr__(self,"position",vector3(self.position))
        if not all(finite(v) for v in (self.yaw,self.pose_age,self.speed)) or min(self.pose_age,self.speed)<0:
            raise ValueError("invalid vehicle feedback")
        if any(type(v) is not bool for v in (self.connected,self.armed,self.landed)):
            raise ValueError("invalid boolean feedback")

@dataclass(frozen=True)
class Decision:
    mode: str
    reference: object
    actions: tuple = ()
    reason: str = ""

def reference_from_pose(position,q,frame_id):
    if frame_id!="map": raise ValueError("setpoint must be in map")
    return Reference(position,quaternion_yaw(q))

class BridgeDecisions:
    def __init__(self,prestream=2.,takeoff_timeout=15.,goal_timeout=2.,max_pose_age=1.,landing_timeout=30.):
        if not all(finite(v) and v>0 for v in (prestream,takeoff_timeout,goal_timeout,max_pose_age,landing_timeout)):
            raise ValueError("invalid bridge timeouts")
        self.prestream,self.takeoff_timeout,self.goal_timeout,self.max_pose_age=prestream,takeoff_timeout,goal_timeout,max_pose_age
        self.mode="IDLE"; self.anchor=self.goal=None; self.goal_at=None
        self.started=self.last_now=None; self.sent={}; self.reason=""
        self.auto_mode="AUTO.LAND"
        self.last_actual=None
        self.landing_timeout=landing_timeout
        self.was_offboard=False
    def command(self,command,feedback,now,height=2.):
        if not finite(now): raise ValueError("invalid bridge clock")
        actual=Reference(feedback.position,feedback.yaw)
        if command in ("LAND","EMERGENCY","RTL"):
            self.anchor,self.mode=actual,"LANDING"
            self.started=now
            self.auto_mode="AUTO.RTL" if command=="RTL" else "AUTO.LAND"
            self.goal=None
        elif command=="HOVER":
            if self.mode!="HOLD": self.anchor=actual
            self.mode="HOLD"
            self.was_offboard=feedback.armed and feedback.mode=="OFFBOARD"
        elif command=="TAKEOFF":
            if self.mode not in ("IDLE","HOLD","FAILED") or not feedback.connected or not feedback.landed or feedback.pose_age>self.max_pose_age:
                return False
            if not finite(height) or height<=0: raise ValueError("invalid takeoff altitude")
            self.anchor=Reference((feedback.position[0],feedback.position[1],height),feedback.yaw)
            self.started,self.mode=now,"PRESTREAM"
            self.sent={}
            self.was_offboard=False
        elif command=="GOTO":
            if self.goal is None or self.goal_at is None or now-self.goal_at>self.goal_timeout: return False
            if not feedback.connected or not feedback.armed or feedback.mode!="OFFBOARD" or feedback.pose_age>self.max_pose_age: return False
            self.mode="TRACK"
            self.was_offboard=True
        elif command=="MOTOR_STOP":
            if not feedback.landed: return False
            self.mode="GROUND_DISARM"
        else: return False
        self.reason=""
        return True
    def set_goal(self,reference,now,feedback,frame_id="map",kind="TRAJECTORY",standoff=1.):
        if not isinstance(reference,Reference) or not finite(now) or frame_id!="map":
            raise ValueError("invalid goal")
        if kind not in ("TRAJECTORY","DIRECT_GOAL"): raise ValueError("invalid goal mode")
        if kind=="DIRECT_GOAL":
            if not finite(standoff) or standoff<0: raise ValueError("invalid standoff")
            delta=tuple(b-a for a,b in zip(feedback.position,reference.position))
            distance=math.sqrt(sum(x*x for x in delta))
            ratio=max(0.,1-standoff/distance) if distance else 0.
            reference=Reference(tuple(a+ratio*d for a,d in zip(feedback.position,delta)),reference.yaw)
        self.goal,self.goal_at=reference,now
    def _fail(self,reason):
        if self.mode!="FAILED" and self.last_actual is not None:
            self.anchor=self.last_actual
        self.mode,self.reason,self.goal="FAILED",reason,None

    def _action(self,name,now):
        if now-self.sent.get(name,float("-inf"))>=.5:
            self.sent[name]=now
            return (name,)
        return ()
    def tick(self,feedback,now):
        if not finite(now): raise ValueError("invalid bridge clock")
        if feedback.connected and feedback.pose_age<=self.max_pose_age:
            self.last_actual=Reference(feedback.position,feedback.yaw)
        if self.last_now is not None and now<self.last_now:
            self._fail("clock_reset")
        self.last_now=now
        if not feedback.connected or feedback.pose_age>self.max_pose_age:
            self._fail("feedback_unavailable")
            return Decision(self.mode,self.anchor,(),self.reason)
        if self.anchor is None: self.anchor=Reference(feedback.position,feedback.yaw)
        if self.mode in ("PRESTREAM","WAIT_ARM","WAIT_OFFBOARD","CLIMB") and now-self.started>=self.takeoff_timeout:
            self._fail("takeoff_timeout")
            return Decision(self.mode,self.anchor,(),self.reason)
        actions=()
        if self.mode=="LANDING" and now-self.started>=self.landing_timeout:
            self._fail("landing_timeout")
            return Decision(self.mode,self.anchor,(),self.reason)
        if self.mode=="HOLD" and self.was_offboard and (not feedback.armed or feedback.mode!="OFFBOARD"):
            self._fail("offboard_lost")
            return Decision(self.mode,self.anchor,(),self.reason)
        if self.mode=="PRESTREAM" and now-self.started>=self.prestream: self.mode="WAIT_ARM"
        if self.mode=="WAIT_ARM":
            if feedback.armed: self.mode="WAIT_OFFBOARD"
            else: actions=self._action("ARM",now)
        if self.mode=="WAIT_OFFBOARD":
            if feedback.mode=="OFFBOARD":
                self.mode="CLIMB"
                self.was_offboard=True
            else: actions=self._action("SET_OFFBOARD",now)
        if self.mode=="CLIMB":
            if not feedback.armed or feedback.mode!="OFFBOARD":
                self._fail("offboard_lost")
            elif feedback.position[2]>=self.anchor.position[2]*.95 and feedback.speed<=.15:
                self.mode="HOLD"
        elif self.mode=="TRACK":
            if not feedback.armed or feedback.mode!="OFFBOARD":
                self._fail("offboard_lost")
            elif self.goal_at is None or now-self.goal_at>self.goal_timeout:
                self.anchor=Reference(feedback.position,feedback.yaw)
                self.mode,self.reason="HOLD","goal_timeout"
            else: return Decision(self.mode,self.goal)
        elif self.mode=="LANDING":
            if feedback.landed:
                if feedback.armed: actions=self._action("DISARM",now)
                else: self.mode="COMPLETE"
            elif feedback.mode!=self.auto_mode: actions=self._action(self.auto_mode,now)
        elif self.mode=="GROUND_DISARM":
            if feedback.landed:
                if feedback.armed: actions=self._action("DISARM",now)
                else: self.mode="COMPLETE"
            else: self._fail("airborne_motor_stop_rejected")
        return Decision(self.mode,self.anchor,actions,self.reason)
