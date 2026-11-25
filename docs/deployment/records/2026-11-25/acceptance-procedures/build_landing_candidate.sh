#!/usr/bin/env bash
# Build only the changed ROS package; reuse the exact tested EGO and PX4 binaries.
set -e -o pipefail
d=/root/autodl-tmp/uav/validation/goal-20261125
old="$d/install04"
new="$d/install05-landing"
bundle="$d/isolated/f04-matrix/landing-fix.bundle"
hash="$1"
[[ "$hash" =~ ^[0-9a-f]{40}$ ]]
test -f "$bundle"
test ! -e "$new"
mkdir "$new"
git clone -q --shared --no-checkout "$old/UAV-navigation" "$new/UAV-navigation"
git -C "$new/UAV-navigation" fetch -q "$bundle" refs/heads/sitl/20261125-landing-confirmation
test "$(git -C "$new/UAV-navigation" rev-parse FETCH_HEAD)" = "$hash"
git -C "$new/UAV-navigation" checkout -q --detach "$hash"
git -C "$new/UAV-navigation" merge-base --is-ancestor 6b58825f8e715d6ac902303b812976ed20693e6e HEAD
source "$old/runtime.env"
ln -s "$old/ego-install" "$new/ego-install"
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH VIRTUAL_ENV CONDA_PREFIX LD_PRELOAD
export LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/lib/x86_64-linux-gnu
source /opt/ros/noetic/setup.bash
source "$old/ego-install/setup.bash"
cd "$new/UAV-navigation/ws"
catkin_make --build "$new/business-build" install -j3 -l4 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_DEVEL_PREFIX="$new/business-devel" -DCMAKE_INSTALL_PREFIX="$new/business-install" > "$new/build.log" 2>&1
umask 077
{
  printf 'export UAV_RUNTIME_HOME=%q\n' "$new"
  printf 'export UAV_ROOT=%q\n' "$new"
  printf 'export UAV_PROJECT_DIR=%q\n' "$new/UAV-navigation"
  printf 'export UAV_EGO_SETUP=%q\n' "$old/ego-install/setup.bash"
  printf 'export UAV_BUSINESS_SETUP=%q\n' "$new/business-install/setup.bash"
  printf 'export UAV_EGO_INSTALL=%q\n' "$old/ego-install"
  printf 'export UAV_BUSINESS_INSTALL=%q\n' "$new/business-install"
  printf 'export PX4_AUTOPILOT_DIR=%q\n' "$PX4_AUTOPILOT_DIR"
  printf 'export UAV_SECRET_ENV=%q\n' "$UAV_SECRET_ENV"
  printf 'export UAV_GUI=1\n'
} > "$new/runtime.env"
UAV_RUNTIME_CONFIG="$new/runtime.env" bash "$new/UAV-navigation/tools/deployment/uav_runtime.sh" check > "$new/check.json"
test -z "$(git -C "$new/UAV-navigation" status --porcelain)"
cat "$new/check.json"
