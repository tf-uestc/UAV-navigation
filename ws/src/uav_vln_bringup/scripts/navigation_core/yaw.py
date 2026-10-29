"""Shortest-angle yaw policy; no ROS imports."""
import math
from .contracts import finite

def angle_delta(target, current):
    if not finite(target) or not finite(current):
        raise ValueError("nonfinite yaw")
    return math.atan2(math.sin(target-current), math.cos(target-current))

def quaternion(yaw):
    if not finite(yaw):
        raise ValueError("nonfinite yaw")
    return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))

def quaternion_yaw(q):
    if len(q) != 4 or not all(finite(x) for x in q):
        raise ValueError("invalid quaternion")
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-9 or abs(norm-1) > .05:
        raise ValueError("nonunit quaternion")
    x,y,z,w = (a/norm for a in q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

class YawPolicy:
    def __init__(self, initial, rate_limit=.8):
        if not finite(initial) or not finite(rate_limit) or rate_limit <= 0:
            raise ValueError("invalid yaw policy")
        self.value, self.rate_limit = float(initial), float(rate_limit)
    def update(self, desired, dt):
        if not finite(dt) or dt < 0:
            raise ValueError("invalid yaw dt")
        if desired is not None:
            limit = self.rate_limit*dt
            self.value += max(-limit, min(limit, angle_delta(desired, self.value)))
        return self.value
    def face(self, position, target, dt):
        dx,dy = target[0]-position[0],target[1]-position[1]
        return self.update(math.atan2(dy,dx) if math.hypot(dx,dy)>1e-6 else None, dt)
