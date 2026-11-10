from pathlib import Path
import json
import math
import sys

FENCE = ((-6, 6), (-4, 4), (1.1, 2.35))


def minimum_distance(a, b, cells):
    vx, vy = b[0] - a[0], b[1] - a[1]
    length2 = vx*vx + vy*vy
    nearest = None
    for x, y, z in cells:
        wx, wy = x - a[0], y - a[1]
        t = max(0., min(1., (wx*vx+wy*vy)/length2)) if length2 else 0.
        distance = math.hypot(wx-t*vx, wy-t*vy)
        if nearest is None or distance < nearest['distance_m']:
            nearest = dict(distance_m=distance, cell=[x,y,z], fraction=t)
    return nearest


def explain(screen):
    start, goal = screen['start'], screen['goal']
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    length = math.hypot(dx, dy)
    output = dict(start=start, goal=goal, distance_m=length, route=screen['route'])
    if length < .3 or abs(goal[2]-start[2]) > .25:
        return dict(output, reason='short_or_height')
    cells = [(x,y,z) for x,y,z in screen['occupied']
             if min(start[2],goal[2])-.2 <= z <= max(start[2],goal[2])+.2
             and min(start[0],goal[0])-1.8 <= x <= max(start[0],goal[0])+1.8
             and min(start[1],goal[1])-1.8 <= y <= max(start[1],goal[1])+1.8]
    direct = minimum_distance(start,goal,cells)
    destination = minimum_distance(goal,goal,cells)
    output.update(cells=len(cells), direct=direct, destination=destination)
    if not direct or direct['distance_m'] >= .13:
        return dict(output, reason='direct_not_blocked')
    if destination['distance_m'] < .08:
        return dict(output, reason='goal_in_inflation')
    nx, ny = -dy/length, dx/length
    candidates = []
    for offset in (1.,-1.,1.2,-1.2,1.4,-1.4,1.6,-1.6):
        first = (start[0]+offset*nx,start[1]+offset*ny,goal[2])
        last = (goal[0]+offset*nx,goal[1]+offset*ny,goal[2])
        inside = all(lo<=v<=hi for point in (first,last) for (lo,hi),v in zip(FENCE,point))
        route = (start,first,last,goal)
        legs = [minimum_distance(a,b,cells) for a,b in zip(route,route[1:])]
        candidates.append(dict(offset=offset, inside=inside, legs=legs,
                               rejected_legs=[i for i,leg in enumerate(legs,1) if leg['distance_m'] < .16]))
    return dict(output, reason='accepted' if screen['route'] else 'no_candidate', candidates=candidates)


def summarize(directory):
    result = json.loads((directory/'results.json').read_text())
    trace = json.loads((directory/'trace.json').read_text())
    start = next((t for t,k,_ in trace if k=='obstacle_inserted'),None)
    events = []
    if start is not None:
        for t,k,d in trace:
            if t<start:
                continue
            if k=='mission' and d['reason'] in ('planner_stop','replan_from_settled_pose','destination_reached'):
                events.append(dict(t=t-start, kind=k, phase=d['phase'], reason=d['reason']))
            if k=='ego_status' and d['status']=='EMERGENCY_STOP':
                events.append(dict(t=t-start, kind=k, reason=d['reason']))
    pairs = []
    before = None
    for line in (directory/'detour.jsonl').read_text().splitlines():
        event = json.loads(line)
        if event['event']=='plan_before':
            before = event
        elif event['event']=='screen' and before and before['require_detour']:
            pairs.append(explain(event))
        elif event['event']=='plan_after' and before and before['require_detour']:
            pairs.append(dict(before=before, after=event))
    return dict(trial=directory.name, passed=result['passed'], error=result['error'],
                final_armed=result['final_armed'], checks=result['checks'], events=events,
                recovery=pairs)


if __name__ == '__main__':
    root = Path(sys.argv[1])
    results = [summarize(path) for path in sorted(root.glob('dynamic-[123]'))
               if all((path/name).is_file() for name in ('results.json','trace.json','detour.jsonl'))]
    (root/'dynamic-analysis.json').write_text(json.dumps(results,indent=2)+'\n')
    for result in results:
        metrics = {item.get('name', item.get('check')):item for item in result['checks']}
        print(json.dumps(dict(trial=result['trial'], passed=result['passed'], final_armed=result['final_armed'],
                              checks=metrics, events=result['events'],
                              recovery=[dict(reason=x.get('reason'), closest=x.get('destination'),
                                             legs=[y['rejected_legs'] for y in x.get('candidates',[])],
                                             map_age_seconds=x.get('before',{}).get('map_age_seconds'),
                                             phase=x.get('after',{}).get('phase')) for x in result['recovery']]),
                         sort_keys=True))
