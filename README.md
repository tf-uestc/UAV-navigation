# UAV-VLN-Navigation

## Windows 离线修复分支状态

本分支正在按 docs/reviews/windows_tasks_2026-10-29.txt 实施。
离线通过不等于 ROS、Gazebo、PX4 飞行验证通过；Ubuntu 清单仍需执行。
不要将旧 README 中的历史运行进度当作本修复分支的仿真验收结果。

在仓库根目录运行：

    python -m pip install -r requirements-offline.txt
    python tools/run_offline_tests.py --log test-results/offline.txt
    python tools/run_offline_replay.py --output test-results/replay.json

测试不访问真实 API，不需要 ROS。核心逻辑使用标准库，图像数组测试使用 NumPy。
每份结果记录实际解释器、NumPy 版本及代码/测试摘要；Python 3.8 实测留 Ubuntu U01。
也可使用 python -m unittest discover -s tests/offline -p "test_*.py" -v。

接口迁移说明：

- 检测业务结果使用 /uav/detection_result 的版本化 JSON String，mission_id/request_id 独立于 Header.seq。
- /uav/target_world 保留为无任务身份的可视化/直接模式输出，任务管理器不再用它驱动接近。
- Path.header.stamp 为绝对起始时间，逐点时间戳也为绝对时间；生产者与跟随器必须同步升级。
- navigation_core/bridge.py 是离线决策核心，尚未替换旧 setpoint_bridge.py。
- 控制唯一发布者、实际 EGO 目标确认/连续重规划、相机标定和飞控接线仍须 Ubuntu U03-U08 验证。
- 深度+odom与点云双路建图问题、EGO 目标 z=1 硬编码尚待实际版本核对；当前不得宣称完整避障闭环已验证。


基于视觉大模型（VLM）的无人机自主搜救导航系统 —— 硕士毕业设计。

## 项目简介

实现无人机在 ROS + Gazebo + PX4 SITL 仿真中的完整导航链路：

```
自然语言指令 → VLM 目标检测 → 3D 定位(TF) → EGO-Planner 轨迹规划 → MAVROS 飞控 → PX4 执行
```

### 核心模块

| 模块 | 实现 | 说明 |
|------|------|------|
| VLM 视觉理解 | `qwen_vl_grounding.py` | Qwen-VL via DashScope API, bbox grounding + 深度融合 |
| 3D 定位 | `detector_node.py` | 像素→相机光学系→map 坐标系（TF 历史时间戳） |
| 轨迹规划 | `ego_adapter.py` → `planner_node.py` | EGO-Planner C++ 节点桥接 |
| 轨迹播放 | `trajectory_follower.py` | 30Hz 线性插值，输出 goal_pose 流 |
| 飞控接口 | `setpoint_bridge.py` | FSM 状态机，MAVROS Offboard 控制（稳定，零改动） |
| 任务管理 | `mission_manager.py` | 持续搜索 FSM（IDLE→SEARCHING→APPROACHING→HOVER），偏航扫描 + 弓字形覆盖 ★ |

## 目录结构

```
├── ws/                                    # ROS workspace
│   └── src/
│       ├── uav_vln_bringup/              # 主功能包
│       │   ├── launch/                    # ROS launch 文件 (8个)
│       │   │   ├── vlm_navigation_full.launch  # 完整链路 (VLM+EGO)
│       │   │   ├── vlm_navigation.launch        # VLM + 直接 setpoint
│       │   │   ├── test_planner_ego.launch      # EGO 集成测试 (无VLM)
│       │   │   ├── test_planner_dummy.launch    # Dummy 规划器测试
│       │   │   ├── ego_planner.launch           # EGO-Planner 核心
│       │   │   ├── mavros_px4_sitl.launch       # MAVROS + TF 配置
│       │   │   ├── mavros_mocap.launch          # MAVROS + 动捕
│       │   │   └── takeoff_land_demo.launch     # 起飞降落演示
│       │   ├── config/
│       │   │   ├── ego_planner_params.yaml
│       │   │   └── mission_params.yaml          # 任务搜索参数
│       │   ├── scripts/
│       │   │   ├── base_detector.py         # Detector ABC + Detection 数据类
│       │   │   ├── qwen_vl_grounding.py     # Qwen-VL 检测器实现
│       │   │   ├── detector_node.py         # 检测 ROS 节点
│       │   │   ├── mission_manager.py       # 任务状态机 + 持续搜索 ★
│       │   │   ├── planner_node.py          # 规划 ROS 节点 (importlib 加载)
│       │   │   ├── trajectory_follower.py   # 轨迹→setpoint 插值播放
│       │   │   ├── setpoint_bridge.py       # 飞控 FSM (不可修改)
│       │   │   ├── tf_healthcheck.py        # 启动 TF 树验证
│       │   │   ├── takeoff_land.py          # 起飞降落演示
│       │   │   └── planning/                # Planner 实现
│       │   │       ├── base.py              # Planner ABC + Trajectory 数据类
│       │   │       ├── ego_adapter.py       # EGO-Planner 桥接
│       │   │       └── dummy_planner.py     # 测试桩
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       └── ego-planner/                    # EGO-Planner (独立 workspace)
├── simulation/                             # 仿真配置
│   └── px4_gazebo_classic/
│       ├── worlds/                         # 自定义 world 文件
│       │   ├── search_rescue.world         # 搜救场景 (起降台 + 红色支架)
│       │   ├── empty.world                 # 空白世界 (fallback)
│       │   └── ...                         # 其他 PX4 自带世界
│       └── models/
│           └── depth_camera/               # Iris 深度相机无人机模型
├── docs/                                   # 文档
├── test/                                   # 测试
├── .env                                    # API Key (gitignored)
├── scripts_start_px4_sitl_gazebo.sh        # PX4 SITL + Gazebo 启动脚本
├── scripts_demo_full.sh                    # 完整 demo 一键启动
├── scripts_demo_all.sh                     # 全部 demo
├── CLAUDE.md                               # AI 助手指引
├── plan.md                                 # 项目计划书
└── README.md
```

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- PX4-Autopilot v1.13.3 (`/home/tf/PX4-Autopilot`)
- Gazebo Classic 11
- DashScope API Key (通义千问)

## 快速开始

### 1. 编译

```bash
cd ws
source /opt/ros/noetic/setup.bash
catkin_make
```

### 2. 启动 PX4 SITL + Gazebo（终端 1）

```bash
cd /home/tf/毕业设计
WORLD=search_rescue ./scripts_start_px4_sitl_gazebo.sh
```

### 3. 编译（首次或修改代码后）

```bash
cd /home/tf/毕业设计/ws
source /opt/ros/noetic/setup.bash
catkin_make
```

### 4. 启动完整导航链路（终端 2）

```bash
cd /home/tf/毕业设计/ws
source /opt/ros/noetic/setup.bash
source src/ego-planner/devel/setup.bash   # ← EGO 必须在主 workspace 之前！
source devel/setup.bash
source /home/tf/毕业设计/.env
roslaunch uav_vln_bringup vlm_navigation_full.launch
```

### 5. 发送指令（终端 3）

```bash
# 起飞
rostopic pub -1 /uav/state_cmd std_msgs/String "data: 'takeoff'"
# VLM 检测 + 导航
rostopic pub -1 /uav/instruction std_msgs/String "data: '{\"target\":{\"text\":\"红色支架\"}}'"
# 降落
rostopic pub -1 /uav/state_cmd std_msgs/String "data: 'land'"
```

### 6. 查看调试图像（终端 4）

```bash
rosrun rqt_image_view rqt_image_view /uav/target_debug
```

## ROS Topic 合约

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/uav/instruction` | String (JSON) | 输入 | 目标描述 `{"target":{"text":"红色支架"}}` |
| `/uav/state_cmd` | String | 输入 | 状态命令 TAKEOFF/LAND/HOVER/RTL/EMERGENCY |
| `/uav/target_world` | PointStamped | 输出 | 检测目标在 map 坐标系中的 3D 位置 |
| `/uav/trajectory` | Path | 输出 | 规划轨迹 (map 坐标系) |
| `/uav/goal_pose` | PoseStamped | 输出 | ≥20Hz setpoint 流，消费于 setpoint_bridge |
| `/uav/target_debug` | Image | 输出 | 标注检测结果的调试图像 |
| `/uav/planner_status` | String | 输出 | 规划器状态 OK/FAIL/PLANNING/MAP_NOT_READY |

## TF 帧链 (REP-105 + REP-103)

```
map → odom → base_link → camera_link → camera_link_optical
```

- `map`: 全局 ENU 坐标系 (east-north-up)
- `camera_link_optical`: ROS 光学约定 (z-forward, x-right, y-down)，VLM 检测结果在此帧

## 模块契约 (ABC + importlib)

所有可替换组件遵循同一模式：抽象基类定义接口，具体实现在子目录中，ROS 节点通过 `importlib` 在运行时加载，由 `~xxx_class` rosparam 指定。**更换 VLM 或规划器无需改动下游代码**，只需修改 launch 文件中的 class 参数。

## 当前进度

| 阶段 | 状态 | 关键产出 |
|------|:----:|----------|
| 环境搭建 + 基础飞控 | ✅ 完成 | PX4 SITL 起飞/悬停/降落闭环 |
| VLM 检测 + 3D 定位 | ✅ 完成 | Qwen-VL grounding → TF → map 坐标 |
| EGO-Planner 集成 | ✅ 完成 | 端到端：指令→检测→规划→飞行 |
| Bug 修复 + 精度提升 | ✅ 完成 | TF 时间戳修正、bbox 归一化、深度融合优化 |
| 状态控制 + 持续搜索 | ✅ 完成 | 偏航扫描 + 弓字形搜索，接近中持续 VLM 修正 |
| Prompt 对比实验 | 🔜 下一步 | "直接返回 bbox" vs "返回像素坐标" 效果对比 |

## 相关项目

- EGO-Planner: [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)
- PX4-Autopilot: [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

## 许可证

本项目仅供学术研究使用。
