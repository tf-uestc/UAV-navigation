# UAV-VLM-Navigation

基于视觉大模型（VLM）的无人机自主搜救导航系统。

## 项目简介

本项目实现了一个完整的无人机自主导航系统，集成以下模块：

- **VLM 视觉理解**：使用视觉大模型识别目标物体/区域
- **EGO-Planner 局部规划**：基于梯度的实时避障轨迹规划
- **PX4 飞控**：工业级飞控，支持仿真与真机部署
- **Gazebo 仿真**：基于 PX4 SITL + Gazebo Classic 的仿真环境

## 系统架构

```
VLM (视觉理解) → 目标点 → EGO-Planner (避障轨迹) → MAVROS → PX4 (飞控) → 无人机
                                                                              ↓
                                                                   深度相机 (感知反馈)
                                                                              ↓
                                                                   VLM (下一帧理解)
```

## 目录结构

```
├── ws/                                    # ROS workspace
│   └── src/
│       └── uav_vln_bringup/              # 主功能包
│           ├── launch/                    # ROS launch 文件
│           │   ├── vlm_navigation.launch  # VLM 导航主启动
│           │   ├── mavros_px4_sitl.launch # MAVROS 启动
│           │   └── takeoff_land_demo.launch
│           ├── scripts/                   # Python 脚本
│           │   ├── vlm_navigation.py      # VLM 导航主程序
│           │   ├── setpoint_bridge.py     # 目标点桥接
│           │   └── takeoff_land.py        # 起飞降落
│           ├── CMakeLists.txt
│           └── package.xml
├── simulation/                            # 仿真配置
│   └── px4_gazebo_classic/
│       ├── worlds/                        # 自定义 world 文件
│       │   └── empty.world
│       └── models/                        # 自定义无人机模型
│           └── iris_depth_camera/
│               └── iris_depth_camera.sdf
├── scripts_start_px4_sitl_gazebo.sh       # PX4 SITL + Gazebo 启动脚本
├── scripts_demo_all.sh                    # 一键启动脚本
├── .gitignore
└── README.md
```

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- PX4-Autopilot v1.13+
- Gazebo Classic 11

## 依赖安装

### 1. PX4-Autopilot

需要单独克隆到上级目录：

```bash
cd ..
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.13.3
make px4_sitl gazebo-classic
```

### 2. EGO-Planner

需要单独克隆到本目录：

```bash
cd /path/to/this/project
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ego-planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 3. ROS 依赖

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

## 快速开始

### 1. 启动 PX4 SITL + Gazebo

```bash
./scripts_start_px4_sitl_gazebo.sh
```

### 2. 启动 ROS 节点

```bash
source ws/devel/setup.bash
roslaunch uav_vln_bringup vlm_navigation.launch
```

## 相关论文

- EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors ([arXiv](https://arxiv.org/abs/2008.08835))
- UAV-VLRR: Vision-Language-Robotics for Search and Rescue

## 许可证

本项目仅供学术研究使用。
