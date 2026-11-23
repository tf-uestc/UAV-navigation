#!/usr/bin/env bash
set -e -o pipefail
runtime_config="$(printenv UAV_RUNTIME_CONFIG || true)"
if test -z "$runtime_config";then
 entry_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
 runtime_config="$entry_dir/runtime.env"
fi
test -r "$runtime_config" || { echo "Missing runtime configuration: $runtime_config" >&2;exit 2; }
export UAV_RUNTIME_CONFIG="$runtime_config"
set -a
source "$runtime_config"
set +a
exec bash "$UAV_PROJECT_DIR/tools/deployment/uav_runtime.sh" "$@"
