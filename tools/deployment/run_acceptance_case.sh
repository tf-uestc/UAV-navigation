#!/bin/bash
set -e -o pipefail
label="$1"
scenario="$2"
distance="$3"
height="$4"
bearing="$5"
expected="$6"
root="${UAV_ACCEPTANCE_DIR:?set UAV_ACCEPTANCE_DIR to this experiment directory}"
d="${UAV_ACCEPTANCE_BUILD:?set UAV_ACCEPTANCE_BUILD to the isolated install root}"
r="$root/flights/$label"
test ! -e "$r" || { echo "Refusing to overwrite a prior flight";exit 2; }
mkdir -p "$r"
export UAV_RUNTIME_CONFIG="$d/runtime.env"
entry="$d/UAV-navigation/tools/deployment/uav_runtime.sh"
bash "$entry" stop > "$r/stop-before.json"
bash "$entry" start > "$r/start.json"
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source /opt/ros/noetic/setup.bash
source "$d/business-install/setup.bash"
source "$d/runtime.env"
export PYTHONPATH="$d/UAV-navigation/tools/deployment:$PYTHONPATH"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
cd "$d/UAV-navigation"
git rev-parse HEAD > "$r/commit.txt"
git status --short > "$r/source-status.txt"
test "$(realpath "$(rospack find ego_planner)")" = "$(realpath "$d/ego-install/share/ego_planner")"
test "$(realpath "$(rospack find uav_vln_bringup)")" = "$(realpath "$d/business-install/share/uav_vln_bringup")"
ground() {
 timeout 6 rostopic echo -n 1 /mavros/state | grep -q '^armed: False'
 timeout 6 rostopic echo -n 1 /mavros/extended_state | grep -q '^landed_state: 1'
}
detect=false
if test "$scenario" = live;then
 detect=true
 source "$UAV_SECRET_ENV"
 test -n "$VLM_API_KEY"
fi
roslaunch uav_vln_bringup segment_verified.launch with_detector:="$detect" standoff:="$distance" height:="$height" require_known_space:=true > "$r/launch.log" 2>&1 &
launch=$!
observer="";recorder="";contacts="";moving=""
cleanup() {
 trap - EXIT
 if ! ground >/dev/null 2>&1;then
  bash "$entry" land > "$r/cleanup-land.json" 2>&1 || true
  for _ in $(seq 1 20);do if ground >/dev/null 2>&1;then break;fi;sleep 1;done
 fi
 if ground >/dev/null 2>&1;then
  kill -TERM "$launch" 2>/dev/null || true;wait "$launch" || true
 else
  echo 'Controller left running because landed state is unverified' >&2
 fi
 for pid in "$observer" "$recorder" "$contacts" "$moving";do
  if test -n "$pid";then kill -TERM "$pid" 2>/dev/null || true;wait "$pid" || true;fi
 done
}
trap cleanup EXIT
if test "$scenario" = live;then
 /usr/bin/python3 -u tools/deployment/observe_target_clearance.py --out "$r/clearance.json" --iris-sdf "$PX4_AUTOPILOT_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf" > "$r/clearance.log" 2>&1 &
 observer=$!
 /usr/bin/python3 -u tools/deployment/record_detection_frames.py --out "$r/images" > "$r/images.log" 2>&1 &
 recorder=$!
 /usr/bin/python3 -u tools/deployment/record_physics_contacts.py --out "$r" --model red_tripod_A > "$r/contacts.log" 2>&1 &
 contacts=$!
 /usr/bin/python3 -u tools/deployment/validate_moving_rgbd.py "$r/moving.json" > "$r/moving.log" 2>&1 &
 moving=$!
 for _ in $(seq 1 40);do if test -f "$r/contacts-ready.json";then break;fi;sleep .25;done
 test -f "$r/contacts-ready.json"
 timeout --signal=INT 805 /usr/bin/python3 -u tools/deployment/validate_search_sitl.py --yaw 0 --bearing "$bearing" --expected "$expected" --standoff "$distance" --mission-timeout 720 --out "$r/result" | tee "$r/run.log"
elif test "$scenario" = unknown;then
 timeout --signal=INT 245 /usr/bin/python3 -u tools/deployment/validate_unknown_sitl.py --out "$r/result" | tee "$r/run.log"
else
 case "$scenario" in
  straight|long|outside|static|dynamic|blocked)
   timeout --signal=INT 360 /usr/bin/python3 -u tools/deployment/validate_planning_sitl.py "$scenario" --repetitions 1 --contacts --out "$r/result" | tee "$r/run.log";;
  *) echo 'Unknown acceptance scenario' >&2;exit 2;;
 esac
fi
ground
if test -n "$moving";then wait "$moving";moving="";fi
echo 0 > "$r/flight.exit"
