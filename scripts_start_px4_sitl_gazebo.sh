#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"

# ========== 配置 ==========
PX4_DIR="${PX4_AUTOPILOT_DIR:-/home/tf/PX4-Autopilot}"
MODEL="${MODEL:-iris_depth_camera}"
WORLD="${WORLD:-empty}"  # without .world suffix

SIM_DIR="$ROOT_DIR/simulation/px4_gazebo_classic"
WORLDS_DIR="$SIM_DIR/worlds"
MODELS_DIR="$SIM_DIR/models"

# ========== 加载 ROS ==========
# 必须加载 ROS，因为相机插件依赖 ROS 库
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "✓ ROS Noetic loaded"
else
    echo "ERROR: ROS Noetic not found at /opt/ros/noetic/setup.bash"
    echo "Camera plugins will fail to load!"
    exit 1
fi

# 加载工作空间的 ROS 配置（如果有）
if [ -f "$ROOT_DIR/ws/devel/setup.bash" ]; then
    source "$ROOT_DIR/ws/devel/setup.bash"
    echo "✓ Workspace ROS packages loaded"
fi

# ========== 检查必要路径 ==========
if [[ ! -d "$PX4_DIR" ]]; then
    echo "ERROR: PX4 directory not found: $PX4_DIR" >&2
    exit 1
fi

if [[ ! -d "$WORLDS_DIR" ]]; then
    echo "ERROR: Worlds directory not found: $WORLDS_DIR" >&2
    exit 1
fi

if [[ ! -f "$WORLDS_DIR/${WORLD}.world" ]]; then
    echo "ERROR: World file not found: $WORLDS_DIR/${WORLD}.world" >&2
    echo "Available worlds:" >&2
    ls -1 "$WORLDS_DIR" | head -20 >&2
    exit 1
fi

# ========== 设置 Gazebo 路径 ==========
# 设置世界文件路径（关键！）
export PX4_SITL_WORLD="$WORLDS_DIR/${WORLD}.world"

# 设置模型路径（让 Gazebo 能找到自定义模型）
if [[ -d "$MODELS_DIR" ]]; then
    export GAZEBO_MODEL_PATH="$MODELS_DIR:${GAZEBO_MODEL_PATH:-}"
    echo "✓ Custom models path: $MODELS_DIR"
    
    # 检查特定模型是否存在
    if [[ -d "$MODELS_DIR/$MODEL" ]]; then
        echo "✓ Using custom model: $MODEL"
    else
        echo "⚠ Model '$MODEL' not found in custom models, will use PX4 default"
    fi
fi

# ========== 设置 ROS 相关环境变量 ==========
# 确保 Gazebo 能找到 ROS 插件（ROS Gazebo 插件在 /opt/ros/noetic/lib/ 下）
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"
# 确保动态链接器能解析 Gazebo 插件的依赖（如 libDepthCameraPlugin.so）
# 注意：sitl_run.sh 会 source setup_gazebo.bash，它会追加 LD_LIBRARY_PATH，
# 所以这里必须确保 /usr/lib/x86_64-linux-gnu/gazebo-11/plugins 在最前面
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

# 设置 ROS 版本（让 sitl_run.sh 知道需要加载 ROS1 的 libgazebo_ros_api_plugin.so）
export ROS_VERSION=1

# ROS 网络配置（如果有多机通信需求可以修改）
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

# ========== 启动 ROS master (roscore) ==========
# libgazebo_ros_api_plugin.so 需要 ROS master 已运行才能成功初始化
if ! rostopic list > /dev/null 2>&1; then
    echo "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    # 等待 roscore 就绪
    sleep 3
    echo "✓ roscore started (PID: $ROSCORE_PID)"
else
    echo "✓ roscore already running"
    ROSCORE_PID=""
fi

# ========== 启用 verbose 模式获取详细日志 ==========
export VERBOSE_SIM=1

# ========== 启动仿真 ==========
cd "$PX4_DIR"

echo ""
echo "=========================================="
echo "Starting PX4 Gazebo Simulation with ROS"
echo "  Model:  $MODEL"
echo "  World:  $WORLD"
echo "  World file: $PX4_SITL_WORLD"
echo "  ROS Master: $ROS_MASTER_URI"
echo "  Verbose: ON (gzserver --verbose)"
echo "=========================================="
echo ""
echo "To view camera topics, in another terminal run:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  rostopic list | grep camera"
echo "  rostopic echo /camera/rgb/image_raw"
echo "  rqt_image_view"
echo "=========================================="
echo ""

# 启动仿真（不使用 exec，以便脚本结束后可以继续运行其他命令）
cmake --build build/px4_sitl_default --target "gazebo-classic_${MODEL}__${WORLD}"

# 仿真结束后清理 roscore
if [ -n "$ROSCORE_PID" ]; then
    echo "Cleaning up roscore..."
    kill $ROSCORE_PID 2>/dev/null || true
fi