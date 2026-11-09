#!/usr/bin/env bash
# PX4 v1.14.0 + project RGB-D model; does not arm or command movement.
set -e -o pipefail
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH
source /opt/ros/noetic/setup.bash
ROOT="${UAV_ROOT:-/root/autodl-tmp/uav}"
PROJECT="${UAV_PROJECT_DIR:-$ROOT/UAV-navigation}"
PX4="${PX4_AUTOPILOT_DIR:-$ROOT/PX4-Autopilot}"
OUT="${UAV_RUN_DIR:-$ROOT/logs/project-rgbd-$(date +%Y%m%d-%H%M%S)}"
source "$ROOT/ego-planner/devel/setup.bash"
source "$PROJECT/ws/devel/setup.bash"
mkdir -p "$OUT"
echo "RUN_DIRECTORY=$OUT"
if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null; then
  echo 'A PX4/Gazebo instance is already running; refusing a duplicate.' >&2
  exit 1
fi
echo $$ > "$OUT/supervisor.pid"
export DISPLAY=:1 XAUTHORITY=/root/.Xauthority
export GAZEBO_MODEL_DATABASE_URI=''
source "$PX4/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4" "$PX4/build/px4_sitl_default"
export GAZEBO_MODEL_PATH="$PROJECT/simulation/px4_gazebo_classic/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib:$LD_LIBRARY_PATH"
export PX4_SIM_MODEL=gazebo-classic_iris_depth_camera
export PX4_SYS_AUTOSTART=10015 PX4_SIM_SPEED_FACTOR=1
export ROS_MASTER_URI=http://localhost:11311 ROS_IP=127.0.0.1
cleanup() {
  trap - EXIT INT TERM
  for pid in ${GUI_PID:-} ${MAVROS_PID:-} ${PX4_PID:-} ${GAZEBO_PID:-} ${MASTER_PID:-}; do
    kill -INT "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if ! rostopic list >/dev/null 2>&1; then
  roscore > "$OUT/roscore.log" 2>&1 &
  MASTER_PID=$!
fi
for attempt in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then break; fi
  sleep 1
done
rosparam set /use_sim_time true
for plugin in libgazebo_ros_openni_kinect.so libgazebo_ros_api_plugin.so; do
  if ldd "/opt/ros/noetic/lib/$plugin" | grep 'not found'; then exit 1; fi
done
/opt/VirtualGL/bin/vglrun -d egl0 gzserver --verbose \
  -s libgazebo_ros_api_plugin.so \
  "$PROJECT/simulation/px4_gazebo_classic/worlds/search_rescue.world" \
  > "$OUT/gzserver.log" 2>&1 &
GAZEBO_PID=$!
/usr/bin/python3 - <<'PY'
import rospy
rospy.init_node('wait_project_gazebo', anonymous=True, disable_signals=True)
rospy.wait_for_service('/gazebo/get_world_properties', timeout=60)
PY
timeout 45 gz model --spawn-file="$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_depth_camera/iris_depth_camera.sdf" \
  --model-name=iris_depth_camera -x 1.01 -y 0.98 -z 0.83 -Y "${UAV_SPAWN_YAW:-0}"
(
  cd "$PX4/build/px4_sitl_default/rootfs"
  exec "$PX4/build/px4_sitl_default/bin/px4" -d "$PX4/build/px4_sitl_default/etc"
) > "$OUT/px4.log" 2>&1 &
PX4_PID=$!
roslaunch uav_vln_bringup mavros_px4_sitl.launch \
  fcu_url:='udp://:14540@127.0.0.1:14580' > "$OUT/mavros.log" 2>&1 &
MAVROS_PID=$!
/opt/VirtualGL/bin/vglrun -d egl0 gzclient --verbose \
  > "$OUT/gzclient.log" 2>&1 &
GUI_PID=$!
printf 'PX4=%s GAZEBO=%s MAVROS=%s GUI=%s\n' "$PX4_PID" "$GAZEBO_PID" "$MAVROS_PID" "$GUI_PID"
wait "$PX4_PID"
