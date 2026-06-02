# F2 VLM Grounding — 实施总结 & 仿真验证指南

最后更新: 2026-05-31 | Commit: `d4cbbd3`

---

## 1. 做了什么

把旧版 `vlm_navigation.py` 里「VLM 口头返回 (u,v) 坐标」的脆弱链路，替换为 **bbox grounding + 深度融合 + TF 世界坐标投影** 的模块化架构。

核心变化:
- 旧链路: VLM → "(u,v)" 文本 → 单像素深度 → 相机投影 → 世界坐标
- 新链路: VLM → bbox 四角坐标 → 框内中位数深度 → 相机投影 → TF→世界坐标

新增 **Detector ABC 抽象层**，可无缝切换不同 VLM 后端:
- `DummyDetector` — 离线测试，返回画面中央固定框，**不需要 VLM API**
- `QwenVLGroundingDetector` — 生产环境，调用 DashScope API 做视觉 grounding

---

## 2. 目录结构

```
ws/src/uav_vln_bringup/
├── CMakeLists.txt                          # 修改: catkin_install_python 加了 detector_node
├── launch/
│   ├── vlm_navigation.launch               # 修改: API key → $(env VLM_API_KEY)，其余不动
│   ├── vlm_navigation_v2.launch            # 新增: v2 启动文件(MAVROS + bridge + detector_node)
│   ├── mavros_px4_sitl.launch              # 不动(F3 已完成)
│   └── takeoff_land_demo.launch            # 不动
├── scripts/
│   ├── base_detector.py                    # 新增: Detection 数据类 + Detector 抽象基类
│   ├── dummy_detector.py                   # 新增: 固定框测试检测器(不用 VLM)
│   ├── qwen_vl_grounding.py                # 新增: Qwen-VL grounding 生产实现
│   ├── detector_node.py                    # 新增: ROS 节点(importlib 加载 + TF + 调试图)
│   ├── setpoint_bridge.py                  # 不动
│   ├── vlm_navigation.py                   # 不动(F2 不碰旧代码)
│   └── takeoff_land.py                     # 不动
└── package.xml                             # 不动

docs/
├── .env.example                            # 新增: 环境变量文档(无真实 key)
├── .gitignore                              # 修改: 加了 .env
└── features/F2_grounding/
    ├── spec.md                             # 预先写好的规格说明
    ├── plan.md                             # 预先写好的实现方案
    ├── task.md                             # 预先写好的任务拆解
    └── implementation.md                   # 本文件
```

---

## 3. 文件说明

### 3.1 `scripts/base_detector.py` (82行) — 契约层

定义了所有检测器必须遵循的接口:

```python
@dataclass
class Detection:
    bbox: Tuple[int,int,int,int]           # 像素框 (x1,y1,x2,y2)
    point_camera: Tuple[float,float,float] # 相机光学系 3D 坐标 (米)
    depth_m: float                          # 深度 (米)
    confidence: float                       # [0,1] 置信度
    raw: dict                               # 原始 API 返回(调试用)
    target_text: str                        # 查询的目标语义
    center: Tuple[int,int]                 # 像素中心(自动从 bbox 计算)

class Detector(ABC):
    def detect(rgb, depth, target_text, cam_info) -> Optional[Detection]
    def healthcheck() -> bool  # 默认返回 True
```

### 3.2 `scripts/dummy_detector.py` (90行) — 测试检测器

**不需要 VLM API**。固定返回画面中央 100×100 像素框，深度取框内中位数。
用于验证 node 骨架 (话题/TF/调试图) 是否正常。

### 3.3 `scripts/qwen_vl_grounding.py` (249行) — 生产检测器

实现 `Detector.detect()` 的完整流程:

1. **API 调用** — 把 RGB 图编码为 base64 JPEG，发到 DashScope OpenAI 兼容接口
2. **Bbox 解析** — 3 种正则兜底:
   - `<|box_start|>(x1,y1),(x2,y2)<|box_end|>` — Qwen2-VL 格式
   - `<box>(x1,y1),(x2,y2)</box>` — Qwen-VL 经典格式
   - `[x1,y1,x2,y2]` — JSON 兜底
3. **归一化判定** — Qwen 可能输出 [0,1000] 归一化坐标或原始像素，自动检测并缩放
4. **深度融合** — 框内有效深度像素 ≥30% 则取中位数，否则返回 None
5. **相机投影** — 像素中心 → 相机光学系 3D 坐标 (K 矩阵反投影)
6. **置信度** — 用框内有效深度比作为 proxy

纯 Python 模块，**不依赖 rospy**，使用 `logging` 记录日志。

### 3.4 `scripts/detector_node.py` (350行) — ROS 节点

ROS 运行时入口，pipeline:

```
/uav/instruction (JSON)
    │
    ▼
_cb_instruction() ──→ 启动 daemon 线程 _run_detection()
    │
    ├── 快照最新 RGB / Depth / CameraInfo
    ├── BGR → RGB 转换 → detector.detect(rgb, depth, target_text, cam_info)
    ├── 拿到 Detection.point_camera → tf_buffer.transform("map")
    ├── 发布 /uav/target_world (PointStamped, frame_id="map")
    └── 发布 /uav/target_debug (Image, 绿框+红十字+文字)
```

关键设计:
- **线程隔离**: 检测跑在 daemon 线程，不阻塞 ROS 回调队列
- **错误兜底**: 任何异常都不崩节点，失败时发带红字的调试图
- **importlib 加载**: `~detector_class` 参数动态导入 detector，切换检测器只需改 launch
- **TF 超时**: 1 秒超时，捕获 `LookupException/ConnectivityException/ExtrapolationException`

默认参数 (全部可 override):
| 参数 | 默认值 |
|------|--------|
| `~detector_class` | `dummy_detector.DummyDetector` |
| `~camera_topic_rgb` | `/iris_depth_camera/camera/rgb/image_raw` |
| `~camera_topic_depth` | `/iris_depth_camera/camera/depth/image_raw` |
| `~camera_topic_info` | `/iris_depth_camera/camera/depth/camera_info` |
| `~target_frame` | `map` |
| `~camera_optical_frame` | `camera_link_optical` |

### 3.5 `launch/vlm_navigation_v2.launch` (77行) — v2 启动文件

```
MAVROS (飞控桥接)
  └── mavros_px4_sitl.launch

setpoint_bridge (飞控接口中间层)
  └── 收发 /uav/goal_pose, /uav/state_cmd

detector_node (VLM 目标检测) ← NEW
  ├── respawn="true"
  ├── detector_class="dummy_detector.DummyDetector" (默认, 可切换)
  └── vlm_api_key="$(env VLM_API_KEY)" (不硬编码)
```

与 v1 并存，老 launch `vlm_navigation.launch` 仍然可用作回退。

---

## 3.6 节点关系与话题架构

```
                                    Gazebo 仿真环境
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    │                     │                     │
                    ▼                     ▼                     ▼
           /iris_depth_camera     /iris_depth_camera     /iris_depth_camera
           /camera/rgb/           /camera/depth/         /camera/depth/
           image_raw              image_raw              camera_info
           (sensor_msgs/Image)    (sensor_msgs/Image)    (sensor_msgs/CameraInfo)
                    │                     │                     │
                    └─────────┬───────────┴──────────┬──────────┘
                              │                      │
                              ▼                      ▼
                    ┌─────────────────────────────────────┐
                    │        detector_node                │
                    │  (新 — F2 核心节点)                  │
                    │                                     │
                    │  订阅:                               │
                    │    /uav/instruction (std_msgs/String) │ ◄── 用户/上层发指令
                    │                                     │
                    │  内部:                               │
                    │    ┌──────────────────────┐         │
                    │    │  DummyDetector        │         │
                    │    │  (默认, 不调 API)      │         │
                    │    │        or             │         │◄── importlib 动态切换
                    │    │  QwenVLGroundingDet.  │         │
                    │    │  (调 DashScope API)   │         │
                    │    └──────────┬───────────┘         │
                    │               │ Detection            │
                    │               ▼                     │
                    │    pixel_to_camera_optical()         │
                    │               │ (x,y,z) 相机光学系    │
                    │    ┌──────────▼───────────┐         │
                    │    │  tf2_ros.Buffer      │         │
                    │    │  transform("map")    │         │
                    │    │  (依赖 F3 TF 链)      │         │
                    │    └──────────┬───────────┘         │
                    │               │                     │
                    │  发布:          │                     │
                    └───────┬───────┼─────────────────────┘
                            │       │
                  ┌─────────▼──┐  ┌─▼──────────────────┐
                  │/uav/       │  │/uav/               │
                  │target_world│  │target_debug        │
                  │(PointStamped│  │(Image, bgr8)       │
                  │ frame=map) │  │ 绿框+红十字+文字     │
                  └──────┬─────┘  └────────────────────┘
                         │
                         │  ← 等待 F4 Planner 消费
                         │
                         ▼
                  ┌──────────────┐
                  │  F4 Planner  │  (未来)
                  │  → /uav/     │
                  │    goal_pose │
                  └──────┬───────┘
                         │
                         ▼
                  ┌──────────────┐
                  │setpoint_bridge│ ← 已有, 不动
                  │ → MAVROS     │
                  │ → PX4 飞控    │
                  └──────────────┘
```

**话题汇总**:

| 话题 | 方向 | 类型 | 节点 |
|------|------|------|------|
| `/iris_depth_camera/camera/rgb/image_raw` | 订阅 | `Image` | Gazebo → detector_node |
| `/iris_depth_camera/camera/depth/image_raw` | 订阅 | `Image` | Gazebo → detector_node |
| `/iris_depth_camera/camera/depth/camera_info` | 订阅 | `CameraInfo` | Gazebo → detector_node |
| `/uav/instruction` | 订阅 | `String` (JSON) | 用户/上层 → detector_node |
| `/uav/target_world` | **发布** | `PointStamped` (map) | detector_node → F4 Planner(未来) |
| `/uav/target_debug` | **发布** | `Image` (bgr8) | detector_node → 调试/可视化 |
| `/uav/goal_pose` | — | — | **F2 不碰**, 留给 F4 |

**TF 帧链** (F3 产物, F2 依赖它):
```
camera_link_optical → camera_link → base_link → odom → map
```

---

## 4. 数据流架构

```
┌─────────────────────────────────────────────────────────────┐
│  detector_node                                              │
│                                                             │
│  /iris_depth_camera/camera/rgb/image_raw  ──► BGR ──► RGB  │
│  /iris_depth_camera/camera/depth/image_raw ──► float32(m)   │
│  /iris_depth_camera/camera/depth/camera_info                │
│                                                             │
│  /uav/instruction ──► {"target":{"text":"red cube"}}       │
│                        │                                    │
│                        ▼                                    │
│              detector.detect(rgb, depth, "red cube", K)     │
│                        │                                    │
│                        ▼                                    │
│              Detection {                                    │
│                bbox: (x1,y1,x2,y2)                         │
│                point_camera: (x,y,z)  ← 相机光学系          │
│                depth_m, confidence                          │
│              }                                              │
│                        │                                    │
│              ┌─────────┴──────────┐                         │
│              ▼                    ▼                         │
│     tf_buffer.transform      cv2.rectangle/circle/putText   │
│     camera_optical→map                                     │
│              │                    │                         │
│              ▼                    ▼                         │
│     /uav/target_world      /uav/target_debug               │
│     (PointStamped, map)    (Image, bgr8)                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. 手动仿真验证步骤

### 前提

- PX4 SITL + Gazebo 已启动 (`./scripts_start_px4_sitl_gazebo.sh`)
- ROS workspace 已 source: `source ws/devel/setup.bash`
- `VLM_API_KEY` 环境变量已设置 (仅 QwenVL 模式需要)

### 5.1 编译

```bash
cd ~/毕业设计/ws
catkin_make
source devel/setup.bash
```

### 5.2 验证 1: Dummy 模式 (不需要 VLM API)

```bash
# 启动 v2 launch
roslaunch uav_vln_bringup vlm_navigation_v2.launch

# 另开终端，发一条检测指令
rostopic pub -1 /uav/instruction std_msgs/String '{"target":{"text":"test"}}'

# 查看调试图像 (应该看到画面中央绿色框+红十字)
rosrun rqt_image_view rqt_image_view /uav/target_debug

# 查看目标点话题
rostopic echo /uav/target_world
```

**期望结果**: 
- `/uav/target_debug` 有绿色矩形框(画面中央 100×100) + 红色十字 + 左上角文字 `target | depth=X.XXm | conf=0.XX | t=X.XXs`
- `/uav/target_world` 有 `PointStamped` 消息，`frame_id="map"`
- 终端日志显示 `[detector] Detected: bbox=...`

### 5.3 验证 2: QwenVL 模式 (需要 VLM API)

```bash
# 先设置 API key
export VLM_API_KEY=sk-你的真实key

# 启动 v2 launch，指定 QwenVL 检测器
roslaunch uav_vln_bringup vlm_navigation_v2.launch \
  detector_class:=qwen_vl_grounding.QwenVLGroundingDetector

# 发检测指令 (目标名字改成你在 Gazebo 里实际放的东西)
rostopic pub -1 /uav/instruction std_msgs/String '{"target":{"text":"red cube"}}'

# 查看调试图像
rosrun rqt_image_view rqt_image_view /uav/target_debug
```

**期望结果**:
- 调试图像上绿色框在**实际目标位置**(不是画面中央)
- 框位置目视正确(这是归一化判定的关键验证)
- 日志显示 `VLM response (xx chars)`

### 5.4 验证 3: 老 launch 回退 (兼容性)

```bash
# 停止 v2，启动 v1
roslaunch uav_vln_bringup vlm_navigation.launch

# 确认 vlm_navigation.py 正常启动，不报 API key 错误
# (旧行为: 等待 bridge 状态 OK 后才开始跑 VLM 循环)
```

### 5.5 验证 4: 异常测试

```bash
# 拔网线/限流测试
# 1. 启动 QwenVL 模式
# 2. 发检测指令，等待 API 调用中
# 3. 拔掉网线
# 期望: 节点不崩溃，调试图显示红字 "DETECTION FAILED: API request failed"
# 4. 插回网线，再发指令
# 期望: 恢复正常
```

### 5.6 查看话题 (完整检查)

```bash
# 看所有 F2 相关话题
rostopic list | grep -E "target_|instruction"

# 应该看到:
#   /uav/target_world
#   /uav/target_debug
#   /uav/instruction

# 检查 TF 链 (F3 产物，F2 依赖它)
rosrun tf tf_echo map camera_link_optical
# 期望: 有连续输出 (不报 "two or more unconnected trees")
```

---

## 6. 验收指标 (来自 spec)

| 指标 | 目标 | 验证方法 |
|------|------|----------|
| bbox 解析成功率 | ≥ 90% | 30 帧 rosbag 统计 `detect() != None` 的比例 |
| 像素中心误差 | ≤ 30 px | 人工标注 vs VLM 框中心，取欧氏距离 |
| 3D 定位 RMSE | ≤ 0.5m(简单) / ≤ 1.0m(中等) | Gazebo 真值 vs `/uav/target_world` 坐标 |
| 单次检测耗时 P95 | ≤ 3s | 日志中 `t=X.XXs` 统计 |
| 框内有效深度比 | ≥ 70% | 调试图文字中 `conf=0.XX` (即有效像素比) |

---

## 7. 已知问题 (Final Wave 审查发现，不阻塞使用)

| 问题 | 影响 | 修复时机 |
|------|------|----------|
| launch 中 `confidence_threshold`/`trigger_rate` 参数未接入 | 低置信度检测仍发布，无频率限制 | T5 验证前修复 |
| `cam_info` 未做 None 检查 | 节点启动瞬间可能多打一条 error 日志 | 下次迭代 |
| `pixel_to_camera_optical` 函数在两处重复 | 代码重复，不影响功能 | 下次重构 |
| dummy 的深度融合缺 30% 阈值 | 测试检测器几乎不会触发 | 非生产路径 |
