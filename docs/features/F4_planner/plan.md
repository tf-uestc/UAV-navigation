# F4 — Plan: Planner 抽象 + EGO-Planner 集成

## 1. 总览

```
F2 (target_world)  +  current_pose (mavros)
       │                       │
       └──────────┬────────────┘
                  ▼
          planner_node.py             ← 通用入口, 加载 ~planner_class
                  │
              ┌───┴────────────────┐
              ▼                    ▼
       EgoPlannerAdapter    SuperPlannerAdapter (future)
              │
              │  内部: 启动 EGO 子进程 / 节点
              │       订阅 /ego_planner_node/...
              │       把 EGO 的 trajectory msg 转成本框架 Trajectory
              ▼
       /uav/trajectory (nav_msgs/Path)
              │
              ▼
       trajectory_follower.py        ← 时间采样 → setpoint
              │
              ▼
       /uav/goal_pose (≥ 20 Hz)      ← 与现状契约一致
              │
              ▼
       setpoint_bridge.py            ← 不动
```

## 2. EGO-Planner 版本与现状

**版本**：`ego-planner` v1（ZJU-FAST-Lab 主仓库 master 分支），**已编译完成并跑通官方 demo**。

不用 v2（ego-planner-swarm）的理由：
- v1 用 PointCloud2 + Odometry 作为最简输入，匹配你现在的话题
- v2 swarm 功能更丰富但调参复杂，毕设没必要
- v2 的话题名与 msg 类型与 v1 不同，adapter 也要重写

**接入方式**：复用已有的 workspace，**不**重新 clone，**不**塞进 `ws/src/`。

约定一个环境变量指向你的 ego-planner workspace 根（带 `devel/` 那一层）：

```bash
# 加到 ~/.bashrc 或者每次启动前 export
export EGO_WORKSPACE=/your/path/to/ego-planner   # ← 改成你实际的路径
```

之后启动顺序固定为：

```bash
source /opt/ros/noetic/setup.bash
source $EGO_WORKSPACE/devel/setup.bash
source /ossfs/workspace/UAV-navigation-main/ws/devel/setup.bash
```

⚠️ **不要把 ego-planner 拷贝/移动到 `ws/src/`**——理由（按重要性）：
1. **失败隔离**：EGO 编译失败不会拖垮你 Python 工作空间的 `catkin_make`
2. **依赖污染**：EGO 自带的消息（`quadrotor_msgs` / `traj_utils`）跟你的包混在一起偶尔会出现奇怪的 import 问题
3. **构建速度**：全量构建从 5-10 秒变成 5-10 分钟；增量构建空 build 也会慢 ~10×
4. **版本切换方便**：将来要试 SUPER / 别的 fork，整个 workspace 删了重建不影响你

## 3. Planner ABC 与 Trajectory

```python
# planning/base.py
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from geometry_msgs.msg import Point

@dataclass
class TrajectoryPoint:
    pos: Tuple[float, float, float]    # x, y, z (米)
    vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    t_from_start: float = 0.0          # 秒

@dataclass
class Trajectory:
    points: List[TrajectoryPoint] = field(default_factory=list)
    frame_id: str = "map"
    stamp_at_start: float = 0.0        # 与 ROS Time 对齐用

class Planner(ABC):
    @abstractmethod
    def plan(self, start: Point, goal: Point) -> Optional[Trajectory]:
        ...
    @abstractmethod
    def is_ready(self) -> bool:
        ...
    def shutdown(self) -> None:
        """清理子进程 / 停止订阅"""
        pass
```

## 4. EgoPlannerAdapter 实现策略

EGO-Planner 不是一个库，是一组节点（`ego_planner_node`、`traj_server`）。`EgoPlannerAdapter` 的工作不是"调用函数"，而是**当一个聪明的中间人**：

```python
# planning/ego_adapter.py
class EgoPlannerAdapter(Planner):
    def __init__(self):
        # 1. 订阅 EGO 的输出
        self.traj_sub = rospy.Subscriber(
            "/ego_planner_node/trajectory",       # 名字按 EGO 实际话题改
            Bspline,                                # EGO 自定义 msg
            self._on_ego_trajectory)

        # 2. 发布 EGO 的输入(目标点)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)

        # 3. 监听地图就绪
        self.map_ready = False
        self.map_sub = rospy.Subscriber(
            "/ego_planner_node/grid_map/occupancy", ..., self._on_map)

        self.last_traj: Optional[Trajectory] = None

    def is_ready(self) -> bool:
        return self.map_ready

    def plan(self, start, goal) -> Optional[Trajectory]:
        # 起点由 EGO 自己读 odom,我们只发 goal
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position = goal
        msg.pose.orientation.w = 1.0
        self.last_traj = None
        self.goal_pub.publish(msg)

        # 等 EGO 给出轨迹
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < deadline and self.last_traj is None:
            rospy.sleep(0.05)
        return self.last_traj

    def _on_ego_trajectory(self, msg):
        # 把 EGO 的 Bspline / MultiDOFJointTrajectory 转 Trajectory
        self.last_traj = self._bspline_to_trajectory(msg)
```

> EGO 实际输出可能是 `quadrotor_msgs/PositionCommand` 或 `traj_msgs/Bspline`（看版本）。第一周做 demo 时跑 `rostopic list | grep ego` 摸清楚。

## 5. planner_node 入口

```python
# planner_node.py
class PlannerNode:
    def __init__(self):
        cls_path = rospy.get_param("~planner_class",
                                    "planning.ego_adapter.EgoPlannerAdapter")
        self.planner = self._load_class(cls_path)

        self.target_sub = rospy.Subscriber(
            "/uav/target_world", PointStamped, self._on_target)
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._on_pose)

        self.traj_pub = rospy.Publisher(
            "/uav/trajectory", Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            "/uav/planner_status", String, queue_size=10, latch=True)

    def _on_target(self, target):
        if not self.planner.is_ready():
            self._publish_status("MAP_NOT_READY")
            return
        self._publish_status("PLANNING")
        traj = self.planner.plan(self.current_pos, target.point)
        if traj is None:
            self._publish_status("FAIL")
            return
        self.traj_pub.publish(self._to_path_msg(traj))
        self._publish_status("OK")
```

## 6. trajectory_follower

最关键的实现细节是**时间对齐**：

```python
# trajectory_follower.py
class TrajectoryFollower:
    def __init__(self):
        self.traj: Optional[Trajectory] = None
        self.traj_start_time: rospy.Time = rospy.Time(0)

        rospy.Subscriber("/uav/trajectory", Path, self._on_traj)
        self.goal_pub = rospy.Publisher("/uav/goal_pose", PoseStamped, queue_size=10)

        rospy.Timer(rospy.Duration(1.0/30.0), self._tick)   # 30 Hz

    def _on_traj(self, msg: Path):
        self.traj = msg
        self.traj_start_time = rospy.Time.now()

    def _tick(self, _):
        if self.traj is None:
            return
        elapsed = (rospy.Time.now() - self.traj_start_time).to_sec()
        # 在 traj.poses 里按时间线性插值; PoseStamped 没有时间戳, 用 index 等距假设
        # (更严格: Trajectory 里带 t_from_start, 转 Path 时塞到 header.stamp)
        idx = self._interpolate_index(elapsed)
        if idx >= len(self.traj.poses):
            sp = self.traj.poses[-1]   # 已到终点, 维持
        else:
            sp = self.traj.poses[idx]
        sp.header.stamp = rospy.Time.now()
        self.goal_pub.publish(sp)
```

⚠️ **细节**：把 `Trajectory.points[i].t_from_start` 塞进 `Path.poses[i].header.stamp` 字段（用 `Time(secs=t_from_start)`），follower 解析时减去 `Time(0)` 拿到秒数。这是 ROS 里偷渡时间戳的惯用招。

## 7. 深度 → 点云

不写 Python，用 ROS 现成的 nodelet：

```xml
<!-- 在 launch 里加 -->
<node pkg="nodelet" type="nodelet" name="depth_to_cloud_manager"
      args="manager"/>
<node pkg="nodelet" type="nodelet" name="depth_to_cloud"
      args="load depth_image_proc/point_cloud_xyz depth_to_cloud_manager">
  <remap from="image_rect"      to="/camera/depth/image_raw"/>
  <remap from="camera_info"     to="/depth/camera_info"/>
  <remap from="points"          to="/camera/depth/points"/>
</node>
```

EGO 的 grid_map 节点配置成订阅 `/camera/depth/points` + `/mavros/local_position/odom`（odom 不是 pose，需要 mavros 也发 odom，mavros 默认会发 `/mavros/local_position/odom`）。

## 8. 端到端 launch 设计

新建 `vlm_navigation_v3.launch`（v2 是只到 F2 的版本，v3 加上 F4）：

```
v3.launch:
  ├─ mavros_px4_sitl.launch
  ├─ tf_healthcheck (F3)
  ├─ depth_to_cloud nodelet
  ├─ <include ego-planner 的 launch>
  ├─ detector_node (F2)
  ├─ planner_node (F4)
  ├─ trajectory_follower
  └─ setpoint_bridge (现状)
```

`<include ego-planner 的 launch>` 需要先 `source $EGO_WORKSPACE/devel/setup.bash`，所以启动用 wrapper script `scripts_demo_v3.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail
: "${EGO_WORKSPACE:?Set EGO_WORKSPACE to your ego-planner workspace path}"

source /opt/ros/noetic/setup.bash
source "$EGO_WORKSPACE/devel/setup.bash"          # 关键, 让 ROS 找到 ego_planner pkg
source /ossfs/workspace/UAV-navigation-main/ws/devel/setup.bash
roslaunch uav_vln_bringup vlm_navigation_v3.launch
```

## 9. 取舍

| 决策 | 选择 | 备选 | 理由 |
|---|---|---|---|
| EGO 集成方式 | 子节点 + 订阅它的输出 | fork 改源码 | 不动上游代码,版本升级容易 |
| 工作空间 | 独立 workspace | submodule 进 ws/src | 编译时间 + 依赖隔离 |
| Trajectory 表示 | List[TrajectoryPoint] | spline 系数 | 通用,不绑定 EGO 内部表示 |
| Path msg 复用 | 用 nav_msgs/Path | 自定义 msg | 标准消息,RViz 直接可视化 |
| Replanning 触发 | 让 EGO 自己触发(它有内置机制) | planner_node 主动调 plan | EGO 已经做了,别重复造 |
| 没有 Planner 时的 fallback | setpoint_bridge 现有的直线段保留 | 删掉 | 安全网,平时不走 |

## 10. 风险（重点）

- **EGO 集成超期**：plan.md 给了 4 周，按 spec.md §8 的四步走，每周交付一个可演示状态。EGO 已经跑通官方 demo（前提风险已消除），但**把它接到你的无人机模型 + 你的话题**这一步仍可能卡 3-7 天，留好缓冲。如果 Week 14 还接不上，立即降级到"A* + 静态地图 + 直线"保毕业。
- **odom 频率不够**：mavros 默认 `/mavros/local_position/odom` 频率 30 Hz，EGO 推荐 50+ Hz。在 `px4_config.yaml` 里改 `local_position.rate_position`。
- **map 帧不匹配**：EGO 默认假设全部在 `world` frame。改 EGO 配置里的 frame 名 → `map`，与 F3 一致。
- **EGO_WORKSPACE 环境变量没设**：wrapper 脚本会直接报错退出（`${EGO_WORKSPACE:?...}`），不会留下半启动的烂摊子。
- **trajectory_follower 抖动**：插值不平滑会让飞机抖。MVP 先用线性插值能跑通就行；如果抖严重，换三次样条或直接吃 EGO 的高频 cmd。
