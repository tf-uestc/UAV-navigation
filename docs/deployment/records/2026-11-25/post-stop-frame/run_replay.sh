#!/bin/bash
set -eu -o pipefail
root="${1:?experiment root}"
d="$(cd "$(dirname "$0")" && pwd -P)"
set +u
source /opt/ros/noetic/setup.bash
source "$root/install03/business-install/setup.bash"
set -u
export ROS_MASTER_URI=http://127.0.0.1:11327 ROS_IP=127.0.0.1
orig="$root/install03/UAV-navigation/ws/src/uav_vln_bringup/scripts"
export PYTHONPATH="$d:$orig:$PYTHONPATH"
if rostopic list > /dev/null 2>&1; then echo 'Dedicated replay master already in use'; exit 2; fi
roscore -p 11327 > "$d/replay-master.log" 2>&1 &
master=$!
trap 'kill -TERM "$master" 2>/dev/null || true; wait "$master" || true' EXIT
for _ in $(seq 1 60); do rostopic list > /dev/null 2>&1 && break; sleep .12; done
rosparam set /use_sim_time true
for version in old new; do
  source_file="$orig/detector_node.py"
  if test "$version" = new; then source_file="$d/detector_node.py"; fi
  set +e
  timeout 40 /usr/bin/python3 "$d/replay_ros.py" "$source_file" "$d/$version-ros.json" \
    _detector_class:=capture_fixture.DetectorFixture _synchronize_camera:=true \
    _managed_requests_only:=true _post_stop_frame_wait:=1.0 > "$d/$version-ros.log" 2>&1
  rc=$?
  set -e
  test -s "$d/$version-ros.json" || { tail -20 "$d/$version-ros.log"; exit 2; }
  if test "$version" = old; then test "$rc" = 1; else test "$rc" = 0; fi
done
