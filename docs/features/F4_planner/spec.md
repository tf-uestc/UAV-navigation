# F4 — Spec: Planner 抽象 + EGO-Planner 集成

## 1. 目标

把 `setpoint_bridge.py:FLYING` 状态里"朝目标方向直线缩进"的弱实现，替换为**带避障的轨迹规划**：F2 给出目标点 → Planner 输出避障轨迹 → trajectory_follower 高频转 setpoint → setpoint_bridge（不变）下发 PX4。

按 `Planner` ABC 抽象第一个实现 `EgoPlannerAdapter`，为后续换 SUPER / Fast-Planner / MARSIM 留空间。

## 2. 范围

In scope:
- `Planner` ABC + `Trajectory` dataclass（见 ARCHITECTURE.md §4.2）
- `EgoPlannerAdapter`：包装上游 EGO-Planner，订阅它的轨迹话题，输出标准 `Trajectory`
- `planner_node.py`：通用入口，按 `~planner_class` 加载实现
- `trajectory_follower.py`：消费 `nav_msgs/Path`，按时间采样发 `/uav/goal_pose` ≥ 20 Hz
- 深度 → 点云的桥接（用 `depth_image_proc` 的 `point_cloud_xyz` nodelet，不写代码）
- 一个完整端到端 launch：F2 → F4 → setpoint_bridge → MAVROS

Out of scope:
- 第二个 Planner 实现（SUPER 等）—— 论文对比章节做
- 全局规划（A* / RRT）—— EGO-Planner 是局部，全局留 future
- 动态障碍物（仿真里先做静态）
- Replanning 频率优化（先用 EGO 默认参数，跑得通再调）

## 3. 输入 / 输出契约

**输入话题**：
- `/uav/target_world` (`PointStamped`, "map") — 来自 F2
- `/mavros/local_position/pose` — 当前位置作为 start
- `/camera/depth/points` (`PointCloud2`, frame=`camera_link_optical`) — 由 `depth_image_proc` 从 `/camera/depth/image_raw` 转出
- `/uav/state_cmd` — 看到 `LAND/RTL/EMERGENCY` 时停止规划

**输出话题**：
- `/uav/trajectory` (`nav_msgs/Path`, frame=`map`) — 完整轨迹（含 stamp）
- `/uav/goal_pose` (`PoseStamped`) — `trajectory_follower` 的输出，setpoint_bridge 兼容
- `/uav/planner_status` (`String`) — `READY` / `PLANNING` / `OK` / `FAIL` / `MAP_NOT_READY`

**Python 接口**（必须实现）：

```python
class Planner(ABC):
    def plan(start: Point, goal: Point) -> Optional[Trajectory]
    def is_ready() -> bool   # 地图是否初始化完毕
```

地图来源由实现自己订阅（EGO 内部已经订阅 PointCloud2 + Odometry 维护占据栅格，不需要 planner_node 处理）。

## 4. 验收指标

对应 `plan.md §4.2` 的"规划与控制指标"：

| 指标 | 旧链路（直线缩进） | F4 验收 |
|---|---|---|
| 简单场景任务成功率（10 次） | 取决于场景 | ≥ 80% |
| 中等场景任务成功率（10 次） | 不能做（直线撞墙） | ≥ 70% |
| 平均到达耗时 | 待测 | 不要求严格基线，但记录 |
| Replanning 平均耗时 | N/A | ≤ 100 ms |
| Replanning 触发次数 / 任务 | N/A | 记录 |
| 最小障碍距离 | N/A | ≥ 0.3 m |
| `/uav/goal_pose` 输出频率 | 现状 ~20 Hz | ≥ 20 Hz（PX4 OFFBOARD 硬要求） |

> 中等场景成功率从"不能做"到"≥70%"是 F4 的核心论文价值，不能省。

## 5. 不变量（最重要）

- **`setpoint_bridge.py` 零改动**。F4 通过 `trajectory_follower` 把轨迹采样成 `/uav/goal_pose`，setpoint_bridge 看到的接口和现在一模一样。这是整个解耦设计的核心兑现。
- **F2 行为可单独跑**：F2 → 直接发 `/uav/goal_pose`（跳过 Planner）的老链路保留，方便回退。
- **OFFBOARD setpoint 流不能断**：F4 失败时 follower 必须发"维持当前位置"的 setpoint，不能停发。

## 6. 风险（这是 plan.md 里点名最高的风险，仔细对待）

| 风险 | 影响 | 兜底 |
|---|---|---|
| EGO-Planner 与你的无人机模型 / 话题名不匹配 | 集成卡 1-2 周 | 先跑 EGO 官方 demo 不动飞机模型 → 再换 |
| EGO 输入的 odom 频率 / frame 不对 | 规划失败、轨迹漂移 | mavros odom 必须 ≥ 50 Hz, frame=`map` |
| 占据栅格初始化前规划被调用 | None | `is_ready()` 守门，未就绪时 planner_node 不调 plan |
| trajectory_follower 时间错位 | 飞行抖 | 用 trajectory header.stamp + `Time.now() - start` 做线性插值 |
| EGO 给出"穿墙"轨迹 | 撞 | 加最小障碍距离软检查；触发就降级悬停 |
| EGO v1 vs v2 选错 | 编译失败 | plan.md §3 选定，统一锁版本 |
| MAVROS 给的 odom 在 `map` 还是 `odom` frame | EGO 接错 | F3 已经统一到 `map`，但要在 EGO 配置里再确认 |

## 7. 与现状的差异 / 迁移

新增节点：
- `planner_node.py`
- `trajectory_follower.py`
- 在 launch 里嵌入 `depth_image_proc/point_cloud_xyz` nodelet

需要外部依赖（plan.md 已写）：
- 在仓库同级或子目录 git clone 一份 `ego-planner`，单独 catkin_make
- 或用 git submodule

`setpoint_bridge.py:FLYING` 那段直线缩进逻辑：**保留**，作为"无 Planner"的回退路径——当 `/uav/trajectory` 长时间没更新，setpoint_bridge 退回直接吃 `/uav/goal_pose` 的行为。

## 8. 阶段性交付

按 plan.md 第 4 阶段（13-16 周），分四步交付，每步都有可演示产物。**EGO-Planner v1 已编译并跑通官方 demo**，所以 Week 13 不再是"装环境"，而是"摸接口"。

1. **Week 13**：摸清 EGO 实际话题 / msg 类型 + 写完 `Planner` ABC + 跑通 dummy planner 端到端
2. **Week 14**：`EgoPlannerAdapter` 跑通，能在仿真里把你飞机的 odom + 一个目标点喂给 EGO，拿回轨迹
3. **Week 15**：trajectory_follower + 端到端 launch，飞行能避一个静态障碍
4. **Week 16**：跑 10 次中等场景，统计指标，写实验章节
