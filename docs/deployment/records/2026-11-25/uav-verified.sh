#!/usr/bin/env bash
# On AutoDL, place this beside the verified November 25 installation directory.
set -eu
experiment_dir="$(cd "$(dirname "$0")" && pwd -P)"
export UAV_RUNTIME_CONFIG="$experiment_dir/install04/runtime.env"
exec bash "$experiment_dir/install04/UAV-navigation/tools/deployment/uav_runtime.sh" "$@"
