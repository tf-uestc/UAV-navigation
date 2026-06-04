# F4 — EGO-Planner 集成任务清单

> **前提状态**：F2（VLM Grounding 感知）与 F3（TF 链）已在开发机上跑通。
> 无人机可通过 `rostopic pub /uav/instruction` 触发 VLM 检测，`detector_node` 输出 `/uav/target_world`（PointStamped, frame_id=map）。
> 当前 `setpoint_bridge.py` 的 FLYING 状态做的是直线缩进——F4 的目标是用 EGO-Planner 替换为带避障的轨迹规划。
>
> **项目代码库**：此仓库为方案与代码参考，实际编译与运行在开发机（另一台电脑）上进行。
>
> **PX4 路径**：开发机默认 `/home/tf/PX4-Autopilot`，启动脚本已支持 `PX4_AUTOPILOT_DIR` 环境变量覆盖。
>
> **EGO-Planner 源码**：`ws/src/ego-planner-master/`，已 clone，需在开发机编译。

---

## 当前完成状态总览

| Feature | 状态 | 说明 |
|---|---|---|
| F2 VLM Grounding | ✅ 已完成 | detector_node + QwenVLGroundingDetector，输出 /uav/target_world |
| F3 TF 链 | ✅ 核心完成 | mavros TF 发布 + 相机静态 TF + tf2_ros.transform() |
| F3 tf_healthcheck | ❌ 未写 | 补充项，不阻塞 F4 |
| F3 动捕 launch | ❌ 未写 | 预留接口，不阻塞 F4 |
| F4 Planner ABC | ❌ 未开始 | 本文档核心 |
| F4 EgoPlannerAdapter | ❌ 未开始 | |
| F4 trajectory_follower | ❌ 未开始 | |
| F4 端到端集成 | ❌ 未开始 | |

---

## EGO-Planner 接口摘要（从源码分析）

> 源码位置：`ws/src/ego-planner-master/src/`

### EGO 核心节点拓扑

```
depth/odom → [grid_map (ego_planner_node 内部)] → 占据栅格
                                                    ↓
/move_base_simple/goal → [waypoint_generator] → /waypoint_generator/waypoints
                                                    ↓
                                              [ego_planner_node FSM]
                                                    ↓
                                           /planning/bspline (ego_planner/Bspline)
                                                    ↓
                                              [traj_server] → /position_cmd (quadrotor_msgs/PositionCommand)
```

### 关键话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/grid_map/odom` | nav_msgs/Odometry | EGO 订阅 | 里程计，remap 到 `/mavros/local_position/odom` |
| `/grid_map/depth` | sensor_msgs/Image | EGO 订阅 | 深度图（与 cloud 二选一） |
| `/grid_map/cloud` | sensor_msgs/PointCloud2 | EGO 订阅 | 点云（与 depth 二选一） |
| `/grid_map/pose` | geometry_msgs/PoseStamped | EGO 订阅 | 相机位姿（depth 模式需要） |
| `/odom_world` | nav_msgs/Odometry | EGO FSM 订阅 | 等同 odom，remap 同上 |
| `/move_base_simple/goal` | geometry_msgs/PoseStamped | waypoint_generator 订阅 | **目标点输入** |
| `/waypoint_generator/waypoints` | nav_msgs/Path | waypoint_generator 发布 | 航点路径 |
| `/planning/bspline` | ego_planner/Bspline | ego_planner_node 发布 | **B样条轨迹输出** |
| `/position_cmd` | quadrotor_msgs/PositionCommand | traj_server 发布 | 100Hz 位置指令（给 SO3 控制器用） |

### Bspline.msg 格式

```
int32 order
int64 traj_id
time start_time
float64[] knots
geometry_msgs/Point[] pos_pts
float64[] yaw_pts
float64 yaw_dt
```

### 关键配置参数（需修改）

| 参数 | 默认值 | 应改为 | 说明 |
|---|---|---|---|
| `grid_map/frame_id` | `world` | `map` | 与 F3 TF 链对齐 |
| `grid_map/pose_type` | 1 | 2 | 2=Odometry 模式 |
| `fsm/flight_type` | 1 | 1 | 1=手动目标（RViz 2D Nav Goal / 话题） |
| `manager/max_vel` | 2.0 | 1.0~2.0 | 视安全需求调低 |
| `manager/max_acc` | 3.0 | 1.5~3.0 | 视安全需求调低 |

### 我们需要的 EGO 子集

EGO 官方 `run_in_sim.launch` 包含仿真器（SO3 控制器 + 四旋翼模拟器），我们 **不需要** 这些。
我们只需：
- `ego_planner_node`（规划核心 + grid_map）
- `waypoint_generator`（目标点管理，manual-lonely-waypoint 模式）

**不需要**：
- `traj_server`（它是 EGO 内部控制器的前端，我们用自己的 `trajectory_follower` + `setpoint_bridge`）
- SO3 控制器、四旋翼模拟器、假深度渲染器（`pcl_render_node`）、假地图生成器（`mockamap`）

---

## 接入策略

```
F2 (target_world)  +  /mavros/local_position/pose (当前位置)
       │                       │
       ▼                       ▼
  planner_node.py              │ (importlib 加载 ~planner_class)
       │                       │
  ┌────┴─────────────────┐    │
  │  EgoPlannerAdapter   │    │
  │  ┌──────────────────┐│    │
  │  │ 发布 goal →      ││    │
  │  │ /move_base_simple/││   │
  │  │ goal             ││    │
  │  │                  ││    │
  │  │ 订阅 ←           ││    │
  │  │ /planning/bspline ││   │
  │  │ (Bspline msg)    ││    │
  │  └──────────────────┘│    │
  │  内部启动:           │    │
  │    ego_planner_node  │←── /camera/depth/points (depth_image_proc 转出)
  │    waypoint_generator│←── /mavros/local_position/odom
  └──────────────────────┘    │
       │                      │
       ▼ /uav/trajectory (nav_msgs/Path)
       │
  trajectory_follower.py
       │
       ▼ /uav/goal_pose (≥20 Hz, PoseStamped)
       │
  setpoint_bridge.py (零改动)
       │
       ▼ MAVROS → PX4
```

**核心设计决策**：

1. **`EgoPlannerAdapter` 直接订阅 `/planning/bspline`**，不走 `traj_server` 的 `/position_cmd`。
   - `traj_server` 是给 EGO 自己的 SO3 控制器用的（带增益参数），我们用 MAVROS Offboard 模式只需位置 setpoint。
   - 我们自己解析 Bspline → 采样为 Path 点序列 → trajectory_follower 按 20+Hz 发 PoseStamped。

2. **`setpoint_bridge.py` 零改动**。F4 通过 `trajectory_follower` 把轨迹转为 `/uav/goal_pose`，bridge 看到的接口不变。

3. **EGO 放在独立 workspace**，`source` 三层 setup.bash。不要把 EGO 拷到 `ws/src/`。

4. **深度用 `depth_image_proc/point_cloud_xyz`** 转点云，不写 Python 代码。

5. **EGO 的 grid_map 配置 `pose_type=2`（Odometry）**，同时订阅 depth + odom，通过 `message_filters::Synchronizer` 时间同步。

---

## Phase 1：F3 收尾（0.5 天，可与 Phase 2 并行）

> F3 核心功能已完成（TF 链工作正常），以下为补齐项。

### 1.1 写 tf_healthcheck.py

- [ ] 创建 `ws/src/uav_vln_bringup/scripts/tf_healthcheck.py`
- 功能：启动时等待 TF 树就绪，检查 3 条关键变换：
  - `map` → `base_link`
  - `base_link` → `camera_link_optical`
  - `map` → `camera_link_optical`
- 任一缺失 → `rospy.logerr` + `sys.exit(1)`
- 超时 10 秒

### 1.2 加入 vlm_navigation.launch

- [ ] 在 `vlm_navigation.launch` 开头加：
  ```xml
  <node pkg="uav_vln_bringup" type="tf_healthcheck.py" name="tf_healthcheck" required="true" output="screen"/>
  ```

### 1.3 写 mavros_mocap.launch（预留，不测）

- [ ] 创建 `ws/src/uav_vln_bringup/launch/mavros_mocap.launch`
- 包含：VRPN client → relay 到 `/mavros/vision_pose/pose` + mavros_px4_sitl.launch
- 注释中说明真机接入时需要的 PX4 参数（`EKF2_AID_MASK`、`EKF2_HGT_MODE`）

### 1.4 更新 CMakeLists.txt

- [ ] `catkin_install_python` 列表添加 `scripts/tf_healthcheck.py`

### 1.5 运行验证

- [ ] 启动仿真 → `rosrun tf tf_echo map camera_link_optical` 确认连续输出
- [ ] 关掉 mavros → tf_healthcheck 报错退出（验证 required=true 生效）

---

## Phase 2：F4 Week 1 — EGO 接口确认 + ABC + Dummy 端到端（2~3 天）

### 2.1 在开发机上确认 EGO 官方 demo 可运行

- [ ] `cd $EGO_WORKSPACE && source devel/setup.bash`
- [ ] `roslaunch ego_planner run_in_sim.launch`
- [ ] 确认 EGO 仿真四旋翼能飞、能避障
- [ ] 录 `rostopic list | grep -E "ego|plan|grid_map|bspline|odom"` 保存到 `docs/features/F4_planner/ego_topic_map.md`
- [ ] 对关键话题 `rostopic info` + `rostopic echo` 记录 msg 类型
- [ ] 重点确认：
  - EGO 输出轨迹话题名和 msg 类型（`/planning/bspline`, ego_planner/Bspline）
  - EGO 订阅的里程计话题（`/grid_map/odom`, nav_msgs/Odometry）
  - EGO 接收目标点的话题（`/move_base_simple/goal`, geometry_msgs/PoseStamped）
  - EGO frame_id 参数（`grid_map/frame_id`，默认 `world`）

### 2.2 创建 planning 目录结构

- [ ] 创建目录：
  ```
  ws/src/uav_vln_bringup/scripts/planning/
  ├── __init__.py
  ├── base.py
  ├── dummy_planner.py
  └── ego_adapter.py       (Phase 3 实现)
  ```
- [ ] `__init__.py` 暴露 `Planner`、`Trajectory`、`TrajectoryPoint`

### 2.3 写 planning/base.py

```python
# planning/base.py
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from geometry_msgs.msg import Point

@dataclass
class TrajectoryPoint:
    pos: Tuple[float, float, float]       # (x, y, z) 米
    vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    t_from_start: float = 0.0             # 秒

@dataclass
class Trajectory:
    points: List[TrajectoryPoint] = field(default_factory=list)
    frame_id: str = "map"
    stamp_at_start: float = 0.0           # ROS Time 对齐

class Planner(ABC):
    @abstractmethod
    def plan(self, start: Point, goal: Point) -> Optional[Trajectory]:
        ...

    @abstractmethod
    def is_ready(self) -> bool:
        """地图初始化完成、可以开始规划"""
        ...

    def shutdown(self) -> None:
        """清理子进程/停止订阅"""
        pass
```

### 2.4 写 planning/dummy_planner.py

- [ ] 实现 `DummyPlanner(Planner)`
- `is_ready()` → 始终返回 `True`
- `plan(start, goal)` → 生成 start→goal 直线插值轨迹（30 个等距点，每点间隔 0.1 秒）
- 先验证 planner_node + trajectory_follower + setpoint_bridge 全链路能跑

### 2.5 写 planner_node.py

- [ ] `ws/src/uav_vln_bringup/scripts/planner_node.py`
- importlib 加载 `~planner_class`（同 detector_node 的模式）
- 订阅：
  - `/uav/target_world` (PointStamped, "map") — 目标点
  - `/mavros/local_position/pose` (PoseStamped) — 当前位姿作为 start
  - `/uav/state_cmd` (String) — LAND/RTL/EMERGENCY 时停止规划
- 发布：
  - `/uav/trajectory` (nav_msgs/Path, frame_id="map") — 规划轨迹
  - `/uav/planner_status` (String, latched) — READY/PLANNING/OK/FAIL/MAP_NOT_READY
- 关键逻辑：
  ```python
  def _on_target(self, msg):
      if not self.planner.is_ready():
          self._publish_status("MAP_NOT_READY")
          return
      self._publish_status("PLANNING")
      start = self.current_pose.pose.position  # Point
      goal = msg.point                         # Point
      traj = self.planner.plan(start, goal)
      if traj is None:
          self._publish_status("FAIL")
          return
      path_msg = self._trajectory_to_path(traj)
      self.traj_pub.publish(path_msg)
      self._publish_status("OK")
  ```
- `Trajectory` → `nav_msgs/Path` 转换：把 `TrajectoryPoint.t_from_start` 塞入 `PoseStamped.header.stamp`（`rospy.Duration(t_from_start)`），follower 据此插值。

### 2.6 写 trajectory_follower.py

- [ ] `ws/src/uav_vln_bringup/scripts/trajectory_follower.py`
- 订阅 `/uav/trajectory` (nav_msgs/Path)
- 发布 `/uav/goal_pose` (PoseStamped) ≥ 20Hz
- 30Hz Timer 实现：
  ```python
  def _tick(self, event):
      if self.traj is None or len(self.traj.poses) == 0:
          # 无轨迹时维持当前位置（不能停发！OFFBOARD 要求）
          self._publish_current_as_setpoint()
          return

      elapsed = (rospy.Time.now() - self.traj_start_time).to_sec()
      # 在 poses 里按 header.stamp (t_from_start) 线性插值
      sp = self._interpolate(elapsed)
      sp.header.stamp = rospy.Time.now()
      sp.header.frame_id = "map"
      self.goal_pub.publish(sp)
  ```
- **空轨迹 / 无轨迹时必须发当前位置 setpoint**，否则 PX4 退出 OFFBOARD
- 轨迹播放完毕（elapsed 超过最后一个点的 t_from_start）→ 保持终点位置
- 新轨迹到来时立即切换（`traj_start_time = rospy.Time.now()`）

### 2.7 写 test_planner_dummy.launch

- [ ] `ws/src/uav_vln_bringup/launch/test_planner_dummy.launch`
- 包含：mavros_px4_sitl.launch + tf_healthcheck + setpoint_bridge + planner_node(detector_class=dummy) + trajectory_follower
- 不含：detector_node、EGO、depth_to_cloud

### 2.8 端到端测试（Dummy 模式）

- [ ] 启动 PX4 SITL + Gazebo
- [ ] `roslaunch uav_vln_bringup test_planner_dummy.launch`
- [ ] 起飞：`rostopic pub -1 /uav/state_cmd std_msgs/String "takeoff"`
- [ ] 发目标：`rostopic pub -1 /uav/target_world geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: 3.0, y: 2.0, z: 2.0}}"`
- [ ] **期望**：飞机沿直线飞到 (3,2,2) 附近悬停
- [ ] 检查 `rostopic hz /uav/goal_pose` ≥ 20Hz
- [ ] 检查 `rostopic echo /uav/planner_status` 输出 OK
- [ ] `rostopic hz /uav/trajectory` 验证轨迹发布

### 2.9 更新 CMakeLists.txt + package.xml

- [ ] `catkin_install_python` 添加：
  ```cmake
  scripts/planner_node.py
  scripts/trajectory_follower.py
  scripts/tf_healthcheck.py
  ```
- [ ] `package.xml` 添加依赖：`nav_msgs`、`message_filters`
- [ ] 编译验证：`cd ws && catkin_make`

**Phase 2 交付物**：Dummy 端到端飞行 demo（detector→planner(dummy)→follower→bridge 全链路通）

---

## Phase 3：F4 Week 2 — EgoPlannerAdapter 实现（3~5 天）

### 3.1 写 planning/ego_adapter.py

- [ ] `EgoPlannerAdapter(Planner)` 核心实现

```python
class EgoPlannerAdapter(Planner):
    def __init__(self):
        # 订阅 EGO 输出
        self.bspline_sub = rospy.Subscriber(
            "/planning/bspline", Bspline, self._on_ego_trajectory)
        # 发布 EGO 目标
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)
        # 监听地图就绪
        self.map_ready = False
        self.odom_received = False
        self._last_traj = None
        # 可选：订阅 grid_map 占据栅格可视化话题判断地图就绪

    def is_ready(self) -> bool:
        return self.odom_received  # 至少收到过一次里程计

    def plan(self, start, goal) -> Optional[Trajectory]:
        # 发目标点给 waypoint_generator
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.position.z = goal.z
        msg.pose.orientation.w = 1.0
        self._last_traj = None
        self.goal_pub.publish(msg)

        # 等 EGO 规划完成（超时 3 秒）
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while rospy.Time.now() < deadline and self._last_traj is None:
            rospy.sleep(0.05)
        return self._last_traj

    def _on_ego_trajectory(self, msg: Bspline):
        self._last_traj = self._bspline_to_trajectory(msg)

    def _bspline_to_trajectory(self, msg: Bspline) -> Trajectory:
        """解析 EGO Bspline msg → Trajectory

        方法：用 EGO 的 pos_pts + knots 构造 UniformBspline，
        按 0.1 秒间隔采样为 TrajectoryPoint 序列。
        需要导入 ego_planner 的 UniformBspline（C++ 编译的 Python 绑定）。

        备选：如果无法导入 UniformBspline，直接对 pos_pts 做线性插值。
        """
        # 实现细节见代码
        ...
```

### 3.2 写 EGO 集成配置

- [ ] 创建 `ws/src/uav_vln_bringup/config/ego_planner_params.yaml`
- 从 EGO 官方 `advanced_param.xml` 提取关键参数，修改：
  ```yaml
  grid_map:
    frame_id: "map"           # ← 从 world 改为 map
    pose_type: 2              # ← Odometry 模式
    resolution: 0.1
    map_size_x: 40.0
    map_size_y: 40.0
    map_size_z: 3.0
    # 深度相机内参（需匹配 iris_depth_camera）
    cx: 320.5
    cy: 240.5
    fx: 554.254
    fy: 554.254
    # 话题 remap 在 launch 里做，不在 yaml

  fsm:
    flight_type: 1            # 手动目标
    thresh_replan: 1.5
    thresh_no_replan: 2.0
    planning_horizon: 7.5     # 1.5 × sensing_horizon
    emergency_time_: 1.0

  manager:
    max_vel: 1.5              # 搜救场景安全优先，低于默认 2.0
    max_acc: 2.0
    planning_horizon: 7.5
  ```
- [ ] **注意**：相机内参 `cx/cy/fx/fy` 需要在开发机上通过 `rostopic echo /iris_depth_camera/camera/depth/camera_info` 获取实际值。

### 3.3 写 EGO 集成 launch

- [ ] 创建 `ws/src/uav_vln_bringup/launch/ego_planner.launch`
- **只包含 EGO 核心节点**（不含仿真器）：
  ```xml
  <launch>
    <arg name="ego_workspace" default="$(env EGO_WORKSPACE)" />

    <!-- 环境检查 -->
    <env name="EGO_WORKSPACE" value="$(arg ego_workspace)" />

    <!-- EGO 规划核心 -->
    <node pkg="ego_planner" name="ego_planner_node" type="ego_planner_node" output="screen">
      <remap from="/odom_world" to="/mavros/local_position/odom"/>
      <remap from="/grid_map/odom" to="/mavros/local_position/odom"/>
      <remap from="/grid_map/depth" to="/iris_depth_camera/camera/depth/image_raw"/>
      <remap from="/grid_map/cloud" to="/camera/depth/points"/>
      <remap from="/grid_map/pose" to="/mavros/local_position/pose"/>
      <rosparam command="load" file="$(find uav_vln_bringup)/config/ego_planner_params.yaml"/>
    </node>

    <!-- 航点生成器 (manual-lonely-waypoint 模式) -->
    <node pkg="waypoint_generator" name="waypoint_generator" type="waypoint_generator" output="screen">
      <remap from="~odom" to="/mavros/local_position/odom"/>
      <remap from="~goal" to="/move_base_simple/goal"/>
      <remap from="~traj_start_trigger" to="/traj_start_trigger"/>
      <param name="waypoint_type" value="manual-lonely-waypoint"/>
    </node>

    <!-- 深度→点云转换 -->
    <node pkg="nodelet" type="nodelet" name="depth_to_cloud_manager" args="manager"/>
    <node pkg="nodelet" type="nodelet" name="depth_to_cloud"
          args="load depth_image_proc/point_cloud_xyz depth_to_cloud_manager">
      <remap from="image_rect" to="/iris_depth_camera/camera/depth/image_raw"/>
      <remap from="camera_info" to="/iris_depth_camera/camera/depth/camera_info"/>
      <remap from="points" to="/camera/depth/points"/>
    </node>
  </launch>
  ```
- **不含**：traj_server、SO3 控制器、四旋翼模拟器、pcl_render_node、mockamap

### 3.4 深度→点云验证

- [ ] 启动 PX4 SITL + Gazebo（iris_depth_camera）
- [ ] 单独启动 depth_to_cloud nodelet
- [ ] `rostopic hz /camera/depth/points` — 期望 ≥ 10Hz
- [ ] RViz 查看 PointCloud2 显示，确认方向和密度合理
- [ ] 确认 `/camera/depth/points` 的 `header.frame_id` = `camera_link_optical`

### 3.5 替换 Dummy → EGO 测试

- [ ] 复制 `test_planner_dummy.launch` → `test_planner_ego.launch`
- [ ] 修改 `planner_class` 为 `planning.ego_adapter.EgoPlannerAdapter`
- [ ] 加入 `<include file="$(find uav_vln_bringup)/launch/ego_planner.launch"/>`
- [ ] 启动后检查：
  - `rostopic echo /uav/planner_status` → READY（odom 已收到）
  - `rostopic hz /planning/bspline` → 发目标后有输出
  - `rostopic hz /uav/trajectory` → 规划成功后有输出

### 3.6 RViz 可视化验证

- [ ] RViz 中添加 `/uav/trajectory`（Path）显示
- [ ] RViz 中添加 EGO 的占用栅格 Marker 显示
- [ ] 手动发目标：`rostopic pub -1 /uav/target_world ...`
- [ ] **对比截图**：Dummy 直线轨迹 vs EGO 避障轨迹

### 3.7 解决 EGO 里程计频率问题

- [ ] 检查 `/mavros/local_position/odom` 频率：`rostopic hz /mavros/local_position/odom`
- [ ] 如果 < 50Hz，修改 mavros 配置：`px4_config.yaml` 中 `local_position:` 下设 `rate_position: 50`（或更高）
- [ ] EGO 的 grid_map 使用 `message_filters::Synchronizer` 同步 depth + odom，频率需匹配

**Phase 3 交付物**：EGO 适配器跑通，RViz 中看到避障轨迹（可不飞飞机）

---

## Phase 4：F4 Week 3 — 端到端避障飞行（3~4 天）

### 4.1 确认 EGO grid_map 正确构建

- [ ] 在 search_rescue.world 场景中启动 EGO
- [ ] RViz 添加 `/grid_map/occupancy` 或 EGO 的 map 可视化
- [ ] 验证红色障碍物（red_tripod_A/B）在占据栅格中正确显示
- [ ] 验证黄色起降台未误标为障碍

### 4.2 trajectory_follower 加固

- [ ] 空轨迹处理：`traj.poses` 为空时发当前位置
- [ ] 时间越界：elapsed 超过最后一个点 → 保持终点
- [ ] 新轨迹替换：立即重置 `traj_start_time`，避免跳帧
- [ ] 频率监控：`rostopic hz /uav/goal_pose` 必须 ≥ 20Hz
- [ ] 偏航角处理：如果 TrajectoryPoint 有 yaw，设置 orientation；否则保持朝向运动方向

### 4.3 写 vlm_navigation_full.launch

- [ ] `ws/src/uav_vln_bringup/launch/vlm_navigation_full.launch`

```
完整链路:
├─ mavros_px4_sitl.launch (MAVROS + TF)
├─ tf_healthcheck (required=true)
├─ depth_to_cloud nodelet (深度→点云)
├─ ego_planner.launch (EGO 核心节点)
├─ detector_node.py (VLM 检测, qwen_vl_grounding)
├─ planner_node.py (planner_class=planning.ego_adapter.EgoPlannerAdapter)
├─ trajectory_follower.py (轨迹→setpoint)
└─ setpoint_bridge.py (飞控接口, 零改动)
```

### 4.4 写启动脚本 scripts_demo_full.sh

```bash
#!/usr/bin/env bash
set -euo pipefail
: "${EGO_WORKSPACE:?Set EGO_WORKSPACE to your ego-planner workspace path}"
: "${VLM_API_KEY:?Set VLM_API_KEY for QwenVL}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
source "$EGO_WORKSPACE/devel/setup.bash"          # EGO workspace
source "$ROOT_DIR/ws/devel/setup.bash"             # UAV workspace

# 启动 PX4 SITL + Gazebo (后台)
"$ROOT_DIR/scripts_start_px4_sitl_gazebo.sh" &
PX4_PID=$!
trap "kill $PX4_PID 2>/dev/null || true" EXIT
sleep 6

# 启动完整导航链路
roslaunch uav_vln_bringup vlm_navigation_full.launch
```

### 4.5 search_rescue.world 端到端避障测试

- [ ] 启动：`WORLD=search_rescue ./scripts_demo_full.sh`
- [ ] 起飞：`rostopic pub -1 /uav/state_cmd std_msgs/String "takeoff"`
- [ ] VLM 检测：`rostopic pub -1 /uav/instruction std_msgs/String '{"target":{"text":"红色支架"}}'`
- [ ] 观察：
  - `/uav/target_debug` 图片中绿色框在目标上
  - `/uav/trajectory` 在 RViz 中绕过障碍
  - 飞机飞向目标并绕过红色支架
  - `/uav/goal_pose` 频率 ≥ 20Hz
- [ ] 录制 rosbag（含 `/tf`, `/uav/*`, `/camera/*`, `/planning/*`）

### 4.6 fallback 测试

- [ ] 测试 EGO 失败 fallback：断开深度话题，planner 返回 FAIL → 飞机应悬停
- [ ] 测试 dummy 切回：修改 launch 参数 `planner_class:=planning.dummy_planner.DummyPlanner` → 直线飞行
- [ ] 确认老 `vlm_navigation.launch` 仍能独立运行

**Phase 4 交付物**：端到端避障飞行视频 + rosbag

---

## Phase 5：F4 Week 4 — 评测 + 文档 + 实验章节（2~3 天）

### 5.1 测试场景设计

- [ ] 简单场景：search_rescue.world，1 个红色支架为目标，起点在起降台
  - 5 个变体：目标在不同方向/距离（前/左/右/远/近）
- [ ] 中等场景：search_rescue.world，在飞行路径上添加额外障碍物
  - 5 个变体：障碍物数量/位置变化
- [ ] 每场景跑 10 次（不同初始偏移），记录结果

### 5.2 写评测脚本

- [ ] `experiments/run_scenario.sh`：自动启动场景、发指令、录 rosbag
- [ ] `experiments/parse_metrics.py`：从 rosbag 提取指标
  - 任务成功率（目标点半径 1.5m 内悬停 = 成功）
  - 平均到达耗时
  - Replanning 次数与耗时（`/uav/planner_status` 时间戳差）
  - 最小障碍距离（从 `/uav/trajectory` 和 grid_map 估算）
  - `/uav/goal_pose` 输出频率

### 5.3 采集数据

- [ ] 简单场景 10 次 × 5 变体 = 50 次飞行
- [ ] 中等场景 10 次 × 5 变体 = 50 次飞行
- [ ] Baseline 对比：切回 DummyPlanner（直线）跑同样的场景

### 5.4 填写 F4 spec 验收表

| 指标 | 旧链路（直线） | F4 验收目标 | 实测 |
|---|---|---|---|
| 简单场景成功率 (10次) | | ≥ 80% | |
| 中等场景成功率 (10次) | 不能做（撞障碍） | ≥ 70% | |
| 平均到达耗时 | | 记录 | |
| Replanning 平均耗时 | N/A | ≤ 100 ms | |
| Replanning 触发次数/任务 | N/A | 记录 | |
| 最小障碍距离 | N/A | ≥ 0.3 m | |
| /uav/goal_pose 输出频率 | ~20 Hz | ≥ 20 Hz | |

### 5.5 解耦验证

- [ ] `diff setpoint_bridge.py` 与 F4 前版本零差异
- [ ] `planner_class=planning.dummy_planner.DummyPlanner` 切回仍能运行
- [ ] 老 `vlm_navigation.launch` 仍能运行（detector→直发 goal_pose→bridge）

### 5.6 文档更新

- [ ] 更新 README.md 目录结构（增加 planning/、config/、scripts_demo_full.sh）
- [ ] 更新 ARCHITECTURE.md 增加 F4 数据流图
- [ ] 写 `docs/features/F4_planner/implementation.md` 实施总结
- [ ] 写论文实验章节草稿（baseline vs EGO 对比表 + 轨迹图）

### 5.7 最终 demo

- [ ] 录 30s 视频：3 个场景（简单→中等→复杂），展示避障效果
- [ ] 存档 rosbag 到 `experiments/rosbags/`

**Phase 5 交付物**：验收表完整 + 论文实验草稿 + 对比视频

---

## 全局完成标准

1. ✅ 简单/中等场景成功率达标
2. ✅ `setpoint_bridge.py` 与 F4 前版本字节级相同
3. ✅ `planner_class=dummy` 切回仍能跑（解耦验证）
4. ✅ 老 `vlm_navigation.launch` 仍能跑（向后兼容）
5. ✅ `/uav/goal_pose` 输出 ≥ 20Hz（PX4 OFFBOARD 硬要求）
6. ✅ EGO 无规划时飞机悬停（不坠机、不断 OFFBOARD）

---

## 风险与应对

| 风险 | 影响 | 应对 |
|---|---|---|
| EGO 与 PX4 话题名/频率不匹配 | 规划失败 | 里程计频率调高到 50Hz+；remap 逐一验证 |
| EGO frame_id 为 `world` 而非 `map` | TF 变换失败 | 配置 `grid_map/frame_id: map` |
| `depth_image_proc` 未安装 | 点云无法生成 | `sudo apt install ros-noetic-depth-image-proc` |
| EGO 编译失败或与主 workspace 消息冲突 | 编译卡住 | EGO 单独 workspace + source overlay |
| Bspline 解析出错（Python 无 C++ 绑定） | 轨迹转换失败 | 降级：对 pos_pts 做线性插值，不走 UniformBspline |
| `traj_server` 的 `/position_cmd` 频率太高 | MAVROS 吃不下 | 不用 traj_server，自己解析 Bspline |
| 一般深度相机有效距离 4.5m，EGO 需调节 `max_ray_length` | 远处障碍不可见 | 调低 `max_ray_length` 和 `planning_horizon` |
| MAVROS odom 默认 30Hz，EGO 期望 50Hz+ | 时间同步差 | 修改 px4_config.yaml rate_position |
| `quadrotor_msgs` 消息包编译冲突 | 两个 workspace 消息不一致 | EGO 单独 workspace，source 顺序：noetic → ego → uav |

---

## 开发机启动环境变量备忘

```bash
# ~/.bashrc 或每次终端手动 export
export PX4_AUTOPILOT_DIR=/home/tf/PX4-Autopilot
export EGO_WORKSPACE=/path/to/ego-planner   # EGO 独立 workspace 路径
export VLM_API_KEY=sk-xxx                    # DashScope API key
source /opt/ros/noetic/setup.bash
source $EGO_WORKSPACE/devel/setup.bash       # EGO workspace（如独立）
source /path/to/UAV-navigation-main/ws/devel/setup.bash
```

EGO 如放在主 workspace 内（选项 A），则只需：
```bash
source /path/to/UAV-navigation-main/ws/devel/setup.bash   # 包含 EGO
```
