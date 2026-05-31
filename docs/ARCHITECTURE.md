# 模块解耦架构

> 这是 F2/F3/F4 三份 spec 的共同前置文档。所有 feature spec 引用这里定义的接口，不重复展开。

## 1. 设计原则

**两层契约**：

1. **进程间** 用 ROS topic / message 类型作为契约。换实现 = 换发布该话题的节点。
2. **进程内** 用 Python 抽象基类（ABC）作为契约。换实现 = 改 launch 里的 `~xxx_class` 参数。

这样切换组件（VLM provider、planner、仿真→动捕）**不需要改下游代码**。

## 2. 目标节点拓扑

```
┌─────────────────┐  /uav/instruction (String, JSON)
│ instruction_    │ ────────────────────────────────►┐
│ parser  (F1)    │                                  │
└─────────────────┘                                  │
                                                     ▼
┌─────────────────┐  /camera/rgb/image_raw   ┌──────────────────┐
│ Gazebo / 真机   │  /camera/depth/image_raw │  detector_node    │ ← 实现 Detector ABC
│ camera          │ ─────────────────────────►│  (F2)             │
└─────────────────┘                          └──────────────────┘
                                                     │
                                                     │ /uav/target_world (PointStamped, "map")
                                                     ▼
┌─────────────────┐                          ┌──────────────────┐
│ /tf  ◄───────────  mavros + camera SDF   │  planner_node     │ ← 实现 Planner ABC
│ map↔base↔camera │                          │  (F4)             │
└─────────────────┘                          └──────────────────┘
                                                     │
                                                     │ /uav/trajectory (nav_msgs/Path)
                                                     ▼
                                              ┌──────────────────┐
                                              │ trajectory_      │
                                              │ follower         │
                                              └──────────────────┘
                                                     │
                                                     │ /uav/goal_pose (PoseStamped) ≥ 20 Hz
                                                     ▼
                                              ┌──────────────────┐
                                              │ setpoint_bridge  │ ← 不变, 与现状兼容
                                              └──────────────────┘
                                                     │
                                                     ▼
                                                MAVROS → PX4
```

## 3. ROS topic 契约

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/uav/instruction` | `std_msgs/String` (JSON 字符串) | F1 | F2, planner_node | 任务 JSON,见 F1 spec |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Gazebo / 真机相机 | F2 | RGB,bgr8 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Gazebo / 真机深度相机 | F2, planner | 单位米,float32 |
| `/depth/camera_info` | `sensor_msgs/CameraInfo` | 相机驱动 | F2 | 内参 |
| `/uav/target_world` | `geometry_msgs/PointStamped` (frame_id="map") | F2 | F4 | F2 输出的目标 3D 点 |
| `/uav/target_debug` | `sensor_msgs/Image` | F2 | rqt_image_view | 框+深度+置信度叠加图 |
| `/uav/trajectory` | `nav_msgs/Path` (frame_id="map") | F4 | trajectory_follower | 已规划轨迹 |
| `/uav/goal_pose` | `geometry_msgs/PoseStamped` (frame_id="map") | trajectory_follower 或 F2 直发 | setpoint_bridge | 现有契约,**不变** |
| `/uav/state_cmd` | `std_msgs/String` | 任意 | setpoint_bridge | 现有,不变 |
| `/uav/bridge_status` | `std_msgs/String` (latched) | setpoint_bridge | 任意 | 现有,不变 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | mavros | F2/F4/follower | 现有,**仿真和动捕都用同一个话题**(动捕通过 `/mavros/vision_pose/pose` 喂给 PX4 后融合发布) |

> ⚠️ MVP 阶段 F4 还没接入时,F2 可以**直发** `/uav/goal_pose`(等价于跳过 planner),保持向后兼容当前 `vlm_navigation.py` 的行为。

## 4. Python ABC 契约

代码组织建议:

```
ws/src/uav_vln_bringup/scripts/
  detection/
    __init__.py
    base.py              # Detector ABC + Detection dataclass
    qwen_vl_grounding.py # Qwen2-VL grounding 实现  (F2 第一个实现)
    grounding_dino.py    # 后续可加,作论文对比
  planning/
    __init__.py
    base.py              # Planner ABC + Trajectory dataclass
    ego_adapter.py       # EGO-Planner 包装  (F4 第一个实现)
    super_adapter.py     # 后续可加
  detector_node.py       # 通用入口,按 ~detector_class 参数实例化
  planner_node.py        # 通用入口,按 ~planner_class 参数实例化
  trajectory_follower.py # 新增,F4 的 follower
  setpoint_bridge.py     # 不动
  vlm_navigation.py      # 逐步弃用,迁移到 detector_node + planner_node
```

### 4.1 Detector ABC

```python
# detection/base.py
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

@dataclass
class Detection:
    bbox: Tuple[int, int, int, int]   # (x1, y1, x2, y2) 像素
    center: Tuple[int, int]            # (u, v) 像素
    depth_m: float                     # 米
    confidence: float                  # [0, 1]
    raw: dict                          # provider 原始返回, 用于调试
    target_text: str                   # 当时查询的目标语义

class Detector(ABC):
    @abstractmethod
    def detect(self,
               rgb: np.ndarray,
               depth: np.ndarray,
               target_text: str,
               camera_info: 'CameraInfo') -> Optional[Detection]:
        """单帧检测;失败返回 None。"""
        ...
```

`detector_node` 负责:
1. 订阅图像/深度/相机内参/instruction;
2. 通过 `~detector_class` 参数 importlib 加载具体实现;
3. 用 TF 把 `Detection` 投影到 `map`,发布到 `/uav/target_world`;
4. 画调试图像发布 `/uav/target_debug`。

**换 detector** = 写一个新的 `Detector` 子类,改 launch 的 `~detector_class` 参数。其他都不动。

### 4.2 Planner ABC

```python
# planning/base.py
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Tuple
from geometry_msgs.msg import Point

@dataclass
class TrajectoryPoint:
    pos: Tuple[float, float, float]
    vel: Tuple[float, float, float]
    yaw: float
    t_from_start: float   # 秒

@dataclass
class Trajectory:
    points: List[TrajectoryPoint]
    frame_id: str = "map"

class Planner(ABC):
    @abstractmethod
    def plan(self,
             start: Point,
             goal: Point,
             # 地图通过 ROS 话题传递,具体话题名由实现自己订阅
             ) -> Optional[Trajectory]:
        ...

    @abstractmethod
    def is_ready(self) -> bool:
        """地图初始化完成、可以开始规划"""
        ...
```

地图来源由 `Planner` 实现自己负责订阅(EGO 自带占据栅格订阅,SUPER 用别的)。`planner_node` 只管"start+goal 进、轨迹出"。

## 5. TF 帧约定 (REP-105 + REP-103)

```
   map          ←─ 全局固定坐标系 (ENU: 东-北-天)
    │
   odom         ←─ 平滑但有漂移; 仿真里和 map 重合
    │
   base_link    ←─ 机体系 (FLU: 前-左-上); mavros 发布
    │
   camera_link  ←─ 相机机械系; SDF / 静态变换发布
    │
   camera_link_optical  ←─ 相机光学系 (z 朝前, x 朝右, y 朝下); ROS 标准, F2 必须用这个 frame
```

**仿真和动捕共用这套 frame**——动捕厂商 SDK(VRPN / NatNet)发布 `/vrpn_client_node/<rb>/pose` 或类似话题,在 launch 里 remap 到 `/mavros/vision_pose/pose`,PX4 融合后照样发布 `/mavros/local_position/pose`。**对 F2/F4 透明**。

详见 `features/F3-tf-frames/spec.md`。

## 6. 启动参数风格

每个"通用入口节点"用 `~impl_class` 字符串选择具体实现:

```xml
<node pkg="uav_vln_bringup" type="detector_node.py" name="detector">
  <param name="detector_class" value="detection.qwen_vl_grounding.QwenVLGroundingDetector"/>
  <param name="vlm_api_key" value="$(env VLM_API_KEY)"/>
  ...
</node>

<node pkg="uav_vln_bringup" type="planner_node.py" name="planner">
  <param name="planner_class" value="planning.ego_adapter.EgoPlannerAdapter"/>
  ...
</node>
```

`importlib.import_module + getattr` 拿到类,实例化后调用接口方法。

## 7. 模块替换示例

| 场景 | 改动 |
|---|---|
| Qwen-VL → GroundingDINO | 写 `detection/grounding_dino.py`,改 launch `~detector_class` |
| EGO → SUPER | 写 `planning/super_adapter.py`,改 launch `~planner_class` |
| 仿真 → 室内动捕 | 改 mavros launch 加 `vision_pose` 配置,Python 代码零改动 |
| 加新的 VLM provider | 写新 Detector 子类即可,API 客户端可以放在 `detection/clients/` 下复用 |

## 8. 不在本架构里解决的问题

- 多机协同(留作 future work)
- 动态目标(只支持单帧静态)
- 实时性保证(没有硬实时,靠话题频率)
