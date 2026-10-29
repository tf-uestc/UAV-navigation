"""Immutable trajectory samples and explicitly scoped playback."""
from dataclasses import dataclass
from typing import Tuple
from .contracts import finite, vector3
from .yaw import angle_delta

@dataclass(frozen=True)
class Sample:
    position: Tuple[float,float,float]
    velocity: Tuple[float,float,float]
    yaw: float
    t: float
    def __post_init__(self):
        object.__setattr__(self,"position",vector3(self.position))
        object.__setattr__(self,"velocity",vector3(self.velocity))
        if not finite(self.yaw) or not finite(self.t) or self.t < 0:
            raise ValueError("invalid sample")

@dataclass(frozen=True)
class TimedTrajectory:
    mission_id: str
    goal_generation: int
    planner_epoch: str
    traj_id: int
    start: float
    points: Tuple[Sample,...]
    frame_id: str = "map"
    def __post_init__(self):
        object.__setattr__(self,"points",tuple(self.points))
        if not self.mission_id or not self.planner_epoch or self.frame_id != "map":
            raise ValueError("invalid context/frame")
        if type(self.goal_generation) is not int or self.goal_generation < 0 or type(self.traj_id) is not int or self.traj_id < 0:
            raise ValueError("invalid identifier")
        if not finite(self.start) or self.start < 0 or not self.points or self.points[0].t != 0:
            raise ValueError("invalid start/first sample")
        if any(a.t>=b.t for a,b in zip(self.points,self.points[1:])):
            raise ValueError("sample times must strictly increase")
    def sample(self, now):
        if not finite(now):
            raise ValueError("invalid clock")
        t = max(0.0,now-self.start)
        if t >= self.points[-1].t:
            p = self.points[-1]
            return Sample(p.position,(0.,)*3,p.yaw,t)
        for a,b in zip(self.points,self.points[1:]):
            if t <= b.t:
                ratio = (t-a.t)/(b.t-a.t)
                mix = lambda x,y: tuple(v+ratio*(w-v) for v,w in zip(x,y))
                return Sample(mix(a.position,b.position),mix(a.velocity,b.velocity),
                    a.yaw+ratio*angle_delta(b.yaw,a.yaw),t)
        return self.points[0]

class TrajectoryBuffer:
    def __init__(self):
        self.context = self.active = self.pending = self.last_now = None
        self.last_id, self.reason = -1, "not_initialized"
    def begin(self, mission_id, generation, epoch):
        if not mission_id or not epoch or type(generation) is not int or generation < 0:
            raise ValueError("invalid playback context")
        self.context = (mission_id,generation,epoch)
        self.active = self.pending = self.last_now = None
        self.last_id,self.reason = -1,"ready"
    def cancel(self):
        self.context = self.active = self.pending = None
        self.reason = "cancelled"
    def accept(self, trajectory, now):
        if not finite(now):
            raise ValueError("invalid clock")
        context = (trajectory.mission_id,trajectory.goal_generation,trajectory.planner_epoch)
        if context != self.context:
            self.reason = "wrong_context"
        elif trajectory.traj_id <= self.last_id:
            self.reason = "old_trajectory"
        elif now > trajectory.start+trajectory.points[-1].t:
            self.reason = "expired"
        else:
            self.last_id = trajectory.traj_id
            if trajectory.start > now:
                self.pending = trajectory
            else:
                self.active,self.pending = trajectory,None
            self.reason = "accepted"
            return True
        return False
    def sample(self, now):
        if not finite(now):
            raise ValueError("invalid clock")
        if self.last_now is not None and now < self.last_now:
            self.cancel()
            self.reason,self.last_now = "clock_reset",now
            return None
        self.last_now = now
        if self.pending is not None and now >= self.pending.start:
            self.active,self.pending = self.pending,None
        return self.active.sample(now) if self.active else None
