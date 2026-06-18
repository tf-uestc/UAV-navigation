# uav_vln_bringup launch files

## 完整导航链路

| Launch 文件 | 说明 | 包含模块 |
|---|---|---|
| `vlm_navigation_full.launch` | **完整链路** (推荐) | MAVROS + TF检查 + EGO-Planner + VLM检测 + 规划 + 轨迹播放 + 飞控 |
| `vlm_navigation.launch` | VLM 检测 + 直接 setpoint | MAVROS + TF检查 + VLM检测 + 飞控 (无规划器) |

## 测试与调试

| Launch 文件 | 说明 |
|---|---|
| `test_planner_ego.launch` | EGO-Planner 集成测试 (需手动发 goal，无 VLM) |
| `test_planner_dummy.launch` | Dummy 规划器测试 (直线轨迹，不需 EGO workspace) |
| `takeoff_land_demo.launch` | 起飞降落演示 (仅飞控) |

## 基础设施

| Launch 文件 | 说明 |
|---|---|
| `mavros_px4_sitl.launch` | MAVROS + TF 静态变换 (map→base_link, base_link→camera→optical) |
| `mavros_mocap.launch` | MAVROS + 动捕系统 (真机使用) |
| `ego_planner.launch` | EGO-Planner 核心节点 + 深度→点云 nodelet |

## 启动顺序

1. 先启动 PX4 SITL + Gazebo: `WORLD=search_rescue ./scripts_start_px4_sitl_gazebo.sh`
2. 再启动导航链路: `roslaunch uav_vln_bringup vlm_navigation_full.launch`

> **注意**: EGO-Planner 是独立 workspace，source 时必须 **先 EGO 后主 workspace**：
> ```bash
> source src/ego-planner/devel/setup.bash   # 先 EGO
> source devel/setup.bash                    # 再主 workspace
> ```
> 顺序反了会导致主 workspace 的包路径被冲掉，roslaunch 找不到 launch 文件。
