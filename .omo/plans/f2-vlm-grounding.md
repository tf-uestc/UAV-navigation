# F2 — VLM Grounding Work Plan

## 决策记录

| # | 问题 | 决定 |
|---|------|------|
| Q1 | 话题前缀 | 写死 `/iris_depth_camera/camera/...` |
| Q2 | 老 launch API key | 老 launch 也改为 `$(env VLM_API_KEY)` |
| Q3 | detection/ 导入 | **方案 A 打平** — 不放子目录，`base.py`/`qwen_vl_grounding.py`/`dummy.py` 直接放 `scripts/` 下 |
| Q4 | Detection 加 point_camera | **同意** — 加 `point_camera: Tuple[float,float,float]` |
| Q5 | /uav/goal_pose | **F2 只发 `/uav/target_world`**，不发 goal_pose |

---

## 文件清单

### 新建（4 个）

| 文件 | 内容 |
|------|------|
| `scripts/base_detector.py` | `Detection` dataclass（含 `point_camera`）+ `Detector` ABC |
| `scripts/dummy_detector.py` | 固定返回画面中央 100×100 框 |
| `scripts/qwen_vl_grounding.py` | Qwen-VL grounding 实现 |
| `scripts/detector_node.py` | ROS 节点入口，importlib 加载 Detector |
| `launch/vlm_navigation_v2.launch` | 新 launch，运行 detector_node |

### 修改（2 个）

| 文件 | 改动 |
|------|------|
| `scripts/vlm_navigation.py` | **只读不改**（F2 不动它） |
| `launch/vlm_navigation.launch` | 第 43 行 API key 改为 `$(env VLM_API_KEY)` |
| `CMakeLists.txt` | `catkin_install_python` 加 `detector_node.py` |

---

## 任务分解

### T0: 环境准备

- [x] T0.1 老 launch API key 移除：`vlm_navigation.launch:43` 改为 `$(env VLM_API_KEY)`
- [x] T0.2 创建 `.env.example` 到 `docs/`，记录 `VLM_API_KEY=sk-xxx`

### T1: Detector ABC + Dummy

- [x] T1.1 写 `scripts/base_detector.py`：`Detection` dataclass（bbox, center, depth_m, confidence, point_camera, raw, target_text）+ `Detector` ABC（detect / healthcheck）
- [x] T1.2 写 `scripts/dummy_detector.py`：固定返回画面中央 100×100 框，深度取框内中位数

### T2: QwenVLGroundingDetector

- [x] T2.1 从 `vlm_navigation.py` 中提取 `VLMApiClient` 的 HTTP 调用逻辑，写入 `scripts/qwen_vl_grounding.py`
- [x] T2.2 实现 `_parse_bbox`：三种正则（Qwen2-VL / Qwen-VL / JSON数组）+ 归一化判定（1000→像素）
- [x] T2.3 实现 `depth_in_bbox`：框内有效像素 ≥30% 则取中位数，否则 None
- [x] T2.4 实现 `pixel_to_camera_optical`：像素→相机光学系 3D 坐标
- [x] T2.5 实现 `Detector.detect()`：API 调用 → bbox 解析 → 深度融合 → 3D 投影 → 返回 `Detection`

### T3: detector_node

- [x] T3.1 写 ROS 节点骨架：订阅 RGB/Depth/CameraInfo/instruction，发布 target_world/target_debug
- [x] T3.2 importlib 加载：`~detector_class` 参数动态导入 detector 类
- [x] T3.3 TF 变换：`tf2_buffer.transform()` 相机系→map 系（F3 产物）
- [x] T3.4 调试图像：绿色 bbox + 红色十字 + 文本（depth/conf/time）

### T4: Launch

- [x] T4.1 新建 `vlm_navigation_v2.launch`：启动 detector_node，参数按 plan §6
- [x] T4.2 加 `respawn="true"` 自动重启
- [x] T4.3 CMakeLists.txt 加 `detector_node.py` 到 `catkin_install_python`

### T5: 验证

- [ ] T5.1 dummy detector 跑通节点：`/uav/target_debug` 有图有框
- [ ] T5.2 QwenVL 跑一帧：**目视确认框位置正确**
- [ ] T5.3 静态 rosbag 30 帧统计解析成功率
- [ ] T5.4 拔网线/超时测试：节点不崩不阻塞
- [ ] T5.5 老 launch 回退测试：`roslaunch ... vlm_navigation.launch` 仍能跑

---

## 验收标准（来自 spec §4）

| 指标 | 目标 |
|------|------|
| bbox 解析成功率 | ≥ 90% |
| 像素中心误差 | ≤ 30 px |
| 3D 定位 RMSE | ≤ 0.5m（简单）/ ≤ 1.0m（中等） |
| 单次检测耗时 P95 | ≤ 3s |
| 框内有效深度比 | ≥ 70% |

---

## 不变量

- F2 崩溃/超时不阻塞 OFFBOARD setpoint 流
- 输出 frame 必须是 `map`
- API key 只从环境变量读
- 老 launch 兼容
