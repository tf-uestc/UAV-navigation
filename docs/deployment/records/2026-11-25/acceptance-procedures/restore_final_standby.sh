#!/usr/bin/env bash
set -eu -o pipefail
d=/root/autodl-tmp/uav/validation/goal-20261125
cd "$d"
test -s T05-evidence-manifest.json
export UAV_RUNTIME_CONFIG="$d/install05-landing/runtime.env"
entry="$d/install05-landing/UAV-navigation/tools/deployment/uav_runtime.sh"
bash "$entry" status > T05-before-stop.json
/usr/bin/python3 -c "import json; a=json.load(open('T05-before-stop.json'))['state']; assert a['connected'] and a['landed'] and not a['armed']"
bash "$entry" stop
for name in px4 gzserver gzclient; do
    if pgrep -x "$name" >/dev/null; then exit 11; fi
done
entry="$d/uav-verified-v04.sh"
bash "$entry" start > final-standby-start.json
bash "$entry" session --standoff 2.4 --height 1.6 > final-standby-session.json
bash "$entry" status > final-standby-status.json
bash "$entry" vnc > final-standby-vnc.json
set +u
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
/usr/bin/python3 - <<'PY'
import json
import rosgraph
with open('final-standby-session.json') as f: session=json.load(f)['mission_status']
with open('final-standby-status.json') as f: status=json.load(f)['state']
publishers=dict(rosgraph.Master('/acceptance_readonly').getSystemState()[0])
owners={k:publishers.get(k) for k in ('/mavros/setpoint_position/local','/uav/goal_pose')}
assert session['phase']=='IDLE' and status['connected'] and status['landed'] and not status['armed'], (session,status)
assert all(nodes==['/flight_executor'] for nodes in owners.values()), owners
with open('final-standby-owners.json','w') as f: json.dump(owners,f,indent=2)
print(json.dumps({'phase':session['phase'],'state':status,'publishers':owners,'vnc':json.load(open('final-standby-vnc.json'))}))
PY
