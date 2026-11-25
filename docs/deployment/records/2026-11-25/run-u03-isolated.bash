#!/usr/bin/env bash
# From the extracted candidate source root. ROS and HTTP are private test fixtures.
set -eo pipefail
root=$(pwd)
out="$root/u03-results"
mkdir -p "$out"
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11316 ROS_IP=127.0.0.1
export ROS_HOME="$root/.ros" ROS_LOG_DIR="$out/ros-logs"
export ROS_PACKAGE_PATH="$root/ws/src:$ROS_PACKAGE_PATH"
export PYTHONPATH="$root/ws/src/uav_vln_bringup/scripts:$PYTHONPATH"
/usr/bin/python3 tools/run_offline_tests.py --pattern test_visual_arrival_depth.py --log "$out/python38-offline.txt"
/usr/bin/python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",11316)); s.close()'
roscore -p 11316 >"$out/master.log" 2>&1 &
master=$!
trap 'kill -INT "$master" 2>/dev/null || true; wait "$master" || true' EXIT
for i in {1..50}; do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep .1
done
rospack find uav_vln_bringup
/usr/bin/python3 tools/deployment/validate_perception.py --fixture \
    --sample docs/deployment/records/2026-11-02/step56-2026-11-02/after \
    --response docs/deployment/records/2026-11-09/step10/fixed-image-response.json \
    --out "$out/perception.json" >"$out/perception.log" 2>&1
