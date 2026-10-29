"""Observation gates and safe-goal geometry; collision checks remain in EGO."""
import math
from .contracts import finite,vector3
from .control import Reference

class TargetEstimator:
    def __init__(self,max_age=30,max_jump=2,replan_distance=.2):
        if not all(finite(v) and v>0 for v in (max_age,max_jump,replan_distance)):
            raise ValueError("invalid target gates")
        self.max_age,self.max_jump,self.replan_distance=max_age,max_jump,replan_distance
        self.point=self.stamp=self.sent=None
        self.reason="empty"
    def reset(self):
        self.point=self.stamp=self.sent=None
    def observe(self,point,stamp,now):
        p=vector3(point)
        if not finite(stamp) or not finite(now) or stamp>now or now-stamp>self.max_age:
            self.reason="stale"; return False
        if self.stamp is not None and stamp<=self.stamp:
            self.reason="old"; return False
        if self.point is not None and math.dist(self.point,p)>self.max_jump:
            self.reason="jump"; return False
        self.point,self.stamp=p,stamp
        self.reason="accepted"
        return True
    def needs_replan(self):
        return self.point is not None and (self.sent is None or math.dist(self.point,self.sent)>=self.replan_distance)
    def mark_sent(self):
        self.sent=self.point

def observation_goal(actual,target,height,standoff,fence):
    position=vector3(actual); point=vector3(target)
    if not finite(height) or not finite(standoff) or standoff<=0:
        raise ValueError("invalid observation goal")
    for axis in "xyz":
        lo,hi=fence[axis+"_min"],fence[axis+"_max"]
        if not finite(lo) or not finite(hi) or lo>=hi: raise ValueError("invalid fence")
    # Ground objects may lie below the permitted aircraft altitude.
    if not (fence["x_min"]<=point[0]<=fence["x_max"] and fence["y_min"]<=point[1]<=fence["y_max"]):
        raise ValueError("target outside horizontal mission area")
    dx,dy=point[0]-position[0],point[1]-position[1]
    length=math.hypot(dx,dy)
    if length<1e-6: raise ValueError("undefined horizontal approach direction")
    goal=(point[0]-standoff*dx/length,point[1]-standoff*dy/length,height)
    if not all(fence[a+"_min"]<=v<=fence[a+"_max"] for a,v in zip("xyz",goal)):
        raise ValueError("observation pose outside fence")
    return Reference(goal,math.atan2(dy,dx))

class ArrivalTracker:
    def __init__(self,position_tolerance=.3,speed_tolerance=.15,dwell=1):
        if not all(finite(v) and v>0 for v in (position_tolerance,speed_tolerance,dwell)):
            raise ValueError("invalid arrival limits")
        self.pos,self.speed,self.dwell=position_tolerance,speed_tolerance,dwell
        self.since=self.last_time=self.goal=None
    def update(self,position,velocity,goal,now):
        p,v,g=vector3(position),vector3(velocity),vector3(goal)
        if not finite(now): raise ValueError("invalid arrival clock")
        reset=self.goal!=g or (self.last_time is not None and now<self.last_time)
        self.goal,self.last_time=g,now
        if reset: self.since=None
        stable=math.dist(p,g)<=self.pos and math.sqrt(sum(x*x for x in v))<=self.speed
        if not stable: self.since=None; return False
        if self.since is None: self.since=now
        return now-self.since>=self.dwell

def lawnmower(center,width,height,spacing,altitude,max_points=1000):
    if not all(finite(v) for v in tuple(center)+(width,height,spacing,altitude)) or min(width,height,spacing)<=0:
        raise ValueError("invalid search pattern")
    passes=max(1,math.ceil(height/spacing))
    if 2*(passes+1)>max_points: raise ValueError("search budget exceeded")
    points=[]
    for i in range(passes+1):
        y=center[1]-height/2+i*height/passes
        xs=(center[0]-width/2,center[0]+width/2)
        if i%2: xs=tuple(reversed(xs))
        points.extend((x,y,altitude) for x in xs)
    return tuple(points)
