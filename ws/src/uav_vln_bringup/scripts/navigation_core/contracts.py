"""Business IDs and observations independent of ROS Header.seq."""
import json
import math
from numbers import Real
from dataclasses import dataclass
from typing import Optional, Tuple

def finite(v):
    return isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v)

def vector3(v):
    if not isinstance(v, (tuple, list)) or len(v) != 3 or not all(finite(x) for x in v):
        raise ValueError("expected three finite numbers")
    return tuple(float(x) for x in v)

@dataclass(frozen=True)
class RequestKey:
    mission_id: str
    request_id: int
    def __post_init__(self):
        if not isinstance(self.mission_id, str) or not self.mission_id.strip():
            raise ValueError("mission_id required")
        if type(self.request_id) is not int or self.request_id < 1:
            raise ValueError("positive request_id required")

@dataclass(frozen=True)
class DetectionResult:
    key: RequestKey
    target_text: str
    status: str
    capture_stamp: float
    completed_stamp: float
    point: Optional[Tuple[float, float, float]] = None
    frame_id: str = "map"
    reason: str = ""
    def __post_init__(self):
        if self.status not in ("SUCCEEDED", "FAILED", "BUSY"):
            raise ValueError("invalid terminal status")
        if not isinstance(self.target_text, str) or not self.target_text.strip():
            raise ValueError("target text required")
        if not all(finite(t) and t >= 0 for t in (self.capture_stamp, self.completed_stamp)):
            raise ValueError("invalid time")
        if self.completed_stamp < self.capture_stamp:
            raise ValueError("observation crosses clock reset")
        if self.frame_id != "map":
            raise ValueError("world observation must be in map")
        if self.status == "SUCCEEDED":
            object.__setattr__(self, "point", vector3(self.point))
        elif self.point is not None:
            raise ValueError("failed observation cannot carry a target")
    def to_json(self):
        return json.dumps(dict(schema_version=1, mission_id=self.key.mission_id,
            request_id=self.key.request_id, target_text=self.target_text, status=self.status,
            capture_stamp=self.capture_stamp, completed_stamp=self.completed_stamp,
            point=self.point, frame_id=self.frame_id, reason=self.reason), allow_nan=False)
    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        if not isinstance(d, dict) or type(d.get("schema_version")) is not int or d["schema_version"] != 1:
            raise ValueError("unsupported observation schema")
        return cls(RequestKey(d["mission_id"], d["request_id"]), d["target_text"],
            d["status"], d["capture_stamp"], d["completed_stamp"],
            d.get("point"), d.get("frame_id", ""), d.get("reason", ""))
