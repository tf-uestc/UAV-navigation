#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$ROOT_DIR/ws"

if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
  echo "Workspace not built: $WS_DIR" >&2
  echo "Build it first: (cd $WS_DIR && catkin_make)" >&2
  exit 1
fi

# Start PX4 SITL+Gazebo in background
"$ROOT_DIR/scripts_start_px4_sitl_gazebo.sh" &
PX4_PID=$!

cleanup() {
  if kill -0 "$PX4_PID" 2>/dev/null; then
    kill "$PX4_PID" || true
  fi
}
trap cleanup EXIT

# Give PX4 some time to open MAVLink port
sleep 6

source "$WS_DIR/devel/setup.bash"
exec roslaunch uav_vln_bringup takeoff_land_demo.launch
