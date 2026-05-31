# F4 — Tasks

按 spec.md §8 的四周节奏走，**每周末必须有可演示产物**——这是这块拖延风险最大、最需要里程碑约束的 feature。

> **当前进展**：EGO-Planner v1 已编译并跑通官方 demo。Week 13 跳过安装，直接进入"摸接口 + 写 ABC"。

## Week 13 — 摸 EGO 接口 + ABC 写完

### 环境固化

- [ ] 把 EGO workspace 路径写到 `~/.bashrc`：`export EGO_WORKSPACE=/your/ego-planner/path`
- [ ] 验证 source 三层 setup.bash 后没冲突：`source /opt/ros/noetic/setup.bash && source $EGO_WORKSPACE/devel/setup.bash && source ws/devel/setup.bash`
- [ ] 在新终端重新跑一次 EGO 官方 demo，确认 source 顺序没破坏现有行为
- [ ] 录一段官方 demo 的 30s 视频存到 `docs/features/F4-planner/screenshots/ego_official_demo.mp4`（论文里"基础组件验证"章节用得上）

### 摸 EGO 实际话题与 msg 类型

这步是 Week 13 真正的工作量所在，**比安装更费事**。

- [ ] 启动官方 demo，跑 `rostopic list | grep -iE "(ego|plan|grid_map|traj|odom)"` 把全部输出复制到 `docs/features/F4-planner/ego_topic_map.md`
- [ ] 对每个相关话题跑 `rostopic info <topic>`，记下 msg 类型（哪个发哪个订）
- [ ] 重点确认四个：
  - [ ] EGO **订阅**的点云话题（多半是 `/grid_map/cloud` 或 `/cloud_in`，看 launch）
  - [ ] EGO **订阅**的里程计话题（多半是 `/grid_map/odom` 或 `/odom_world`）
  - [ ] EGO **接收目标点**的话题（多半是 `/move_base_simple/goal` 或 `/waypoint_generator/waypoints`）
  - [ ] EGO **输出轨迹**的话题与 msg（可能是 `quadrotor_msgs/PositionCommand` 或 `traj_utils/Bspline`）
- [ ] `rostopic echo` 一下输出轨迹 msg，看字段长什么样
- [ ] 把"EGO 期望的 frame_id"也记下来（grep launch 里的 `world_frame` / `frame_id` 参数）

### Planner ABC

- [ ] 在 `ws/src/uav_vln_bringup/scripts/planning/` 新建目录与 `__init__.py`
- [ ] 写 `planning/base.py`：`Planner` ABC + `Trajectory` / `TrajectoryPoint` dataclass
- [ ] 写 `planning/dummy.py`：固定返回直线轨迹，先把 follower / planner_node 骨架跑通
- [ ] 加 `tf2_ros`、`nav_msgs`、`pcl_ros` 到 `package.xml` 依赖

### Dummy 端到端打通（这是 Week 13 的可演示产物）

- [ ] 写最小 `planner_node.py`（dummy 实现就够）
- [ ] 写最小 `trajectory_follower.py`
- [ ] 写 `planner_dummy.launch`：mavros + dummy planner + follower + setpoint_bridge
- [ ] 起飞 → 手动 `rostopic pub /uav/target_world ...` → 飞机沿直线飞到目标
- [ ] **这一步还不涉及 EGO**，但验证整套抽象 + follower 流程跑得通

**Week 13 末交付**：① 官方 demo 视频 ② `ego_topic_map.md` 接口清单 ③ ABC + dummy 端到端飞行 demo（一段直线但走完了 detector→planner→follower→bridge 全链路）。

## Week 14 — EgoPlannerAdapter 替换 dummy

### Adapter 实现

- [ ] 写 `planning/ego_adapter.py`（plan.md §4 骨架）
- [ ] 实现 `_bspline_to_trajectory`（按 Week 13 摸到的 EGO 实际 msg 类型；可能需要 `import quadrotor_msgs.msg` 或 `traj_utils.msg`）
- [ ] 实现 `is_ready`：监听 EGO 的 grid_map 状态话题
- [ ] 写单测：构造一个 mock EGO msg，验证转 `Trajectory` 字段对得上

### 切换 planner_class

- [ ] 把 `planner_dummy.launch` 复制成 `planner_ego.launch`
- [ ] 改 `~planner_class` 为 `planning.ego_adapter.EgoPlannerAdapter`
- [ ] 在同一个 launch 里 `<include>` 你的 EGO launch（或者 wrapper script 单独起 EGO）
- [ ] 跑 `rostopic hz /uav/trajectory` 看出来的轨迹更新频率

### RViz 验证

- [ ] 在 RViz 里同时显示 `/uav/trajectory`（Path）和 EGO 自己的轨迹可视化 marker，确认一致
- [ ] 手动 `rostopic pub /uav/target_world ...` 一个绕过障碍的目标，看出来的 trajectory 是不是真的避障了 → **截图**
- [ ] 对比 Week 13 dummy 的"直线轨迹" vs Week 14 的"避障轨迹"，截两张图放一起

**Week 14 末交付**：在 RViz 里看到 EGO 输出避障路径，且这条路径走的是你的 `Planner` ABC 接口（说明抽象工作）。无需上飞机。

## Week 15 — 真实深度 + 端到端避障飞行

> Week 13/14 都没用你飞机的相机，EGO 吃的是它 demo 自带的模拟点云。Week 15 真正把感知接进 EGO。

### 深度 → 点云

- [ ] 在 launch 里加 `depth_image_proc/point_cloud_xyz` nodelet（plan.md §7）
- [ ] 跑 `rostopic hz /camera/depth/points` 看频率（应 ≥ 10 Hz）
- [ ] 在 RViz 里加 PointCloud2 显示，看点云是否合理（朝向、密度、frame_id 是 `camera_link_optical`）

### EGO 配置改造

- [ ] 复制 EGO 的默认配置 yaml 到 `ws/src/uav_vln_bringup/config/ego_config.yaml`
- [ ] 改 `world_frame_id: map`（与 F3 一致）
- [ ] 改 grid_map 订阅话题为 `/camera/depth/points` 和 `/mavros/local_position/odom`
- [ ] 启动后看 EGO 日志，确认接到点云、接到里程计
- [ ] grid_map 在 RViz 显示出来（栅格颜色 = 占据），看障碍物是否在正确位置

### trajectory_follower 加固

- [ ] Week 13 的 follower 是最简版本，这里补边界处理：空轨迹、时间越界、连续替换轨迹时的平滑切换
- [ ] 让 follower 在轨迹更新时**保留当前位置作为新轨迹的起点**，避免抖跳

### 端到端 launch

- [ ] 写 `vlm_navigation_v3.launch`（plan.md §8 结构）
- [ ] 写 wrapper 脚本 `scripts_demo_v3.sh`（按 plan.md §8 模板，依赖 `EGO_WORKSPACE` 环境变量）
- [ ] 在 `empty.world` 加一个固定立方体障碍
- [ ] 起飞 → 让 F2 给目标 → 看飞机绕过立方体到达
- [ ] 录视频

**Week 15 末交付**：端到端避障飞行视频（真实深度，不是 EGO 自带模拟）；rosbag 存档。

## Week 16 — 评测 + 实验章节

### 测试场景

- [ ] 简单场景：1 障碍 1 目标（5 个变体）
- [ ] 中等场景：3-5 障碍 + 干扰物 + 目标在视野边缘（5 个变体）
- [ ] 每个场景跑 10 次（不同初始随机种子）

### 指标采集

- [ ] 写 `experiments/run_scenario.sh`：自动启动场景、记录 rosbag、写 CSV
- [ ] 写 `experiments/parse_metrics.py`：从 rosbag 抽取指标（任务成功率、replanning 时间、最小障碍距离）
- [ ] 输出表格 + 折线图

### 论文素材

- [ ] 在 `docs/experiments/F4_results.md` 填 spec.md §4 验收表
- [ ] 写实验章节草稿（baseline = 直线缩进, treatment = EGO）
- [ ] 录最终 demo 视频（30s，3 个场景）

**Week 16 末交付**：spec.md §4 表全部填好，论文实验章节草稿，对比视频。

## 全局完成标准

- 简单 / 中等场景成功率达标
- `setpoint_bridge.py` 与 F4 之前的版本**字节级相同**（`diff` 验证解耦兑现）
- `~planner_class=planning.dummy.DummyPlanner` 切到 dummy 仍能跑（接口验证）
- 老 launch (`vlm_navigation.launch`、`vlm_navigation_v2.launch`) 仍能跑
