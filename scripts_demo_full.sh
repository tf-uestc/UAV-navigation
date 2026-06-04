#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# scripts_demo_full.sh — 完整 VLM + EGO 避障导航一键启动
#
# 要求环境变量:
#   EGO_WORKSPACE  — EGO-Planner workspace root (含 devel/)
#   PX4_AUTOPILOT_DIR — PX4-Autopilot 路径 (默认 /home/tf/PX4-Autopilot)
#   VLM_API_KEY    — DashScope API key (可通过 .env 加载)
#
# 使用方法:
#   source .env
#   WORLD=search_rescue ./scripts_demo_full.sh
# =============================================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$ROOT_DIR/ws"
PX4_DIR="${PX4_AUTOPILOT_DIR:-/home/tf/PX4-Autopilot}"
EGO_DIR="${EGO_WORKSPACE:-}"

# --- 环境变量检查 ---
if [[ -z "${VLM_API_KEY:-}" ]]; then
    if [[ -f "$ROOT_DIR/.env" ]]; then
        source "$ROOT_DIR/.env"
    else
        echo "WARNING: VLM_API_KEY not set and .env not found" >&2
    fi
fi

# --- 检查必要路径 ---
for d in "$PX4_DIR" "$WS_DIR"; do
    if [[ ! -d "$d" ]]; then
        echo "ERROR: directory not found: $d" >&2
        exit 1
    fi
done

# --- 加载 ROS ---
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"       # 含 EGO (已在 ws/src/ 内)

# --- 启动 PX4 SITL + Gazebo (后台) ---
"$ROOT_DIR/scripts_start_px4_sitl_gazebo.sh" &
PX4_PID=$!
cleanup() { kill "$PX4_PID" 2>/dev/null || true; }
trap cleanup EXIT

echo "Waiting for PX4 + Gazebo..."
sleep 8

# --- 启动完整导航链路 ---
exec roslaunch uav_vln_bringup vlm_navigation_full.launch
