#!/usr/bin/env bash
set -e -o pipefail
runtime_config="$(printenv UAV_RUNTIME_CONFIG || true)"
if test -z "$runtime_config";then runtime_config=/root/autodl-tmp/uav/runtime/runtime.env;fi
test -r "$runtime_config" || { echo "Missing runtime configuration: $runtime_config" >&2;exit 2; }
set -a
source "$runtime_config"
set +a
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_PRELOAD
export LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/lib/x86_64-linux-gnu
source /opt/ros/noetic/setup.bash
test -r "$UAV_EGO_SETUP" && test -r "$UAV_BUSINESS_SETUP"
source "$UAV_EGO_SETUP"
source "$UAV_BUSINESS_SETUP"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
if test "$1" = session;then
 test -r "$UAV_SECRET_ENV"
 set -a
 source "$UAV_SECRET_ENV"
 set +a
 test -n "$VLM_API_KEY"
fi
exec /usr/bin/python3 "$UAV_PROJECT_DIR/tools/deployment/uav_runtime.py" "$@"
