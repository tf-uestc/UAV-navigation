"""Single-owner control decisions. ROS publishing is connected in U06."""
from dataclasses import dataclass
from .contracts import finite,vector3
from .trajectory import TrajectoryBuffer

@dataclass(frozen=True)
class Reference:
    position: tuple
    yaw: float
    def __post_init__(self):
        object.__setattr__(self,"position",vector3(self.position))
        if not finite(self.yaw): raise ValueError("nonfinite heading")

class ControlArbiter:
    def __init__(self):
        self.buffer=TrajectoryBuffer()
        self.mode="IDLE"; self.anchor=None; self.live=False; self.safety=False
    def begin(self,mission,generation,epoch,actual):
        if self.safety: raise ValueError("safety stop must be explicitly cleared")
        self.buffer.begin(mission,generation,epoch)
        self.mode,self.anchor,self.live="HOLD",actual,True
    def track(self,trajectory,now):
        if not self.live or self.safety: return False
        if self.buffer.accept(trajectory,now):
            self.mode="TRACK"; return True
        return False
    def hold(self,actual):
        # Capture once per entry, not once per new measured pose.
        if self.mode!="HOLD": self.anchor=actual
        self.mode="HOLD"
        self.buffer.active=self.buffer.pending=None
    def pause_detection(self,actual,pause=True):
        if pause: self.hold(actual)
    def cancel(self,actual):
        self.buffer.cancel()
        self.mode,self.anchor,self.live="HOLD",actual,False
    def stop(self,actual):
        self.cancel(actual); self.safety=True; self.mode="SAFETY_HOLD"
    def clear_safety(self):
        self.safety=False
    def output(self,now):
        if self.mode=="TRACK":
            p=self.buffer.sample(now)
            if p is not None: return Reference(p.position,p.yaw)
            self.live=False; self.mode="HOLD"
        return self.anchor if self.mode in ("HOLD","SAFETY_HOLD") else None
