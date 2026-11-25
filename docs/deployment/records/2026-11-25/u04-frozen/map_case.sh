#!/bin/bash
set -e -o pipefail
mode="$1"
yaw="$2"
b=/root/autodl-tmp/uav/validation/goal-20261125/install03
r=/root/autodl-tmp/uav/validation/goal-20261125/isolated/u04-final03/guards
o="$r/$mode-$yaw"
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH
source /opt/ros/noetic/setup.bash
source "$b/business-install/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11318 ROS_IP=127.0.0.1
cd "$b/UAV-navigation"
if rostopic list >/dev/null 2>&1; then echo 'test port occupied';exit 1;fi
roscore -p 11318 > "$o.master.log" 2>&1 &
master=$!;launch=""
trap 'if [ -n "$launch" ];then kill -TERM "$launch" 2>/dev/null || true;wait "$launch" || true;fi;kill -TERM "$master" 2>/dev/null || true;wait "$master" || true' EXIT
for _ in $(seq 1 40);do if rostopic list >/dev/null 2>&1;then break;fi;sleep .2;done
if [ "$mode" = readiness ];then rosparam set /use_sim_time true;fi
roslaunch uav_vln_bringup planner_verified.launch require_known_space:=true with_camera_bridge:=false obstacle_inflation:=0.349 > "$o.launch.log" 2>&1 &
launch=$!
if [ "$mode" = readiness ];then
 timeout --signal=INT 100 /usr/bin/python3 -u tools/deployment/validate_map_readiness.py --out "$o.json" | tee "$o.log"
else
 timeout --signal=INT 100 /usr/bin/python3 -u tools/deployment/validate_known_start.py --yaw "$yaw" --out "$o.json" | tee "$o.log"
fi
