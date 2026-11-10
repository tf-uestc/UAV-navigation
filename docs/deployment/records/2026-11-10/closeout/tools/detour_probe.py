import json
import os
import runpy
import sys
import time
from pathlib import Path

script = sys.argv.pop(1)
sys.argv[0] = script
namespace = runpy.run_path(script, run_name='detour_probe_target')
mission = namespace['SegmentMission']
plan = mission.plan_segment
module = plan.__globals__
original_detour = module['local_detour']
log = Path(os.environ['DETOUR_PROBE_OUTPUT'])


def record(event, **data):
    with log.open('a') as stream:
        stream.write(json.dumps(dict(wall=time.monotonic(), event=event, **data), allow_nan=False)+'\n')


def inspect_detour(start, goal, occupied, inside):
    cells = tuple(occupied)
    route = original_detour(start, goal, cells, inside)
    record('screen', start=start, goal=goal, occupied=cells, route=route)
    return route


def inspect_plan(self, kind, require_detour=False):
    status = self.planner_status()
    snapshot = self.occupancy
    record('plan_before', mission_id=self.mission, kind=kind, require_detour=require_detour,
           final_goal=self.final_goal, goal_id=self.goal_id, detour=self.detour,
           planner_ready=bool(status), planner_session=status.get('planner_session_id'),
           map_session=snapshot[2] if snapshot else None,
           map_age_seconds=time.monotonic()-snapshot[1] if snapshot else None,
           map_points=snapshot[0].width*snapshot[0].height if snapshot else None)
    plan(self, kind, require_detour)
    record('plan_after', mission_id=self.mission, phase=self.phase, reason=self.reason,
           goal=self.goal, goal_id=self.goal_id, detour=self.detour)


module['local_detour'] = inspect_detour
mission.plan_segment = inspect_plan
mission().run()
