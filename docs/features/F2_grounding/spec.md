# F2 — Spec: VLM Grounding 感知

## 1. 目标

把当前 `vlm_navigation.py` 里"VLM 口头返回 (u,v)"的脆弱链路，替换为 **基于 VLM grounding bbox 的目标 3D 定位模块**，并按 `Detector` ABC 实现，使 VLM provider 与整个感知方案都可换。

## 2. 范围

In scope:
- 输入 RGB + Depth + 目标语义文本 → 输出目标在 `map` 系下的 3D 点
- 一种 baseline 实现：`QwenVLGroundingDetector`
- 调试图像发布（带框、深度、置信度）
- 单帧检测，失败返回 None；上层决定是否重试

Out of scope（留 future work，由别的 feature 处理）:
- 多帧滤波 / Kalman（写在 plan.md 风险章节，不阻塞 F2 验收）
- 动态目标跟踪
- 第二种 detector 实现（如 GroundingDINO）—— 论文对比章节再做
- TF 链修正（由 F3 负责，F2 只调用 tf2 接口，不关心内部）

## 3. 输入 / 输出契约

参见 [ARCHITECTURE.md §3 §4.1](../../ARCHITECTURE.md)。具体本 feature 会用到的：

**输入话题**：
- `/camera/rgb/image_raw` (bgr8)
- `/camera/depth/image_raw` (float32, 米)
- `/depth/camera_info`
- `/uav/instruction` (String, JSON; 取 `.target.text` 字段)
- `/tf` (依赖 F3 完成)

**输出话题**：
- `/uav/target_world` (`PointStamped`, frame_id=`"map"`)
- `/uav/target_debug` (`Image`, bgr8)

**Python 接口**（必须实现）：

```python
class Detector(ABC):
    def detect(rgb, depth, target_text, camera_info) -> Optional[Detection]
```

`Detection` 结构见 ARCHITECTURE.md §4.1。

## 4. 验收指标

直接对应 `plan.md §4.2` 的"感知指标"。每条用同一组测试场景统计：

| 指标 | 现版 baseline | F2 验收 |
|---|---|---|
| 解析成功率（VLM 给出可用 bbox 的比例） | 待测 | ≥ 90% |
| 像素中心误差（vs 人工标注） | 待测 | ≤ 30 px (848×480 图像) |
| 3D 定位 RMSE（vs Gazebo 真值） | 待测 | ≤ 0.5 m（简单场景），≤ 1.0 m（中等场景） |
| 单次检测耗时（含 API 网络） | 待测 | ≤ 3 s（P95） |
| 框内有效深度比 | 待测 | ≥ 70%（≥ 70% 像素既非 NaN 也非 0） |

> baseline = 当前 `vlm_navigation.py` 用 "(u,v)" 文本输出的版本。F2 完成后跑同一组测试，把对比表写进论文。

测试集要求：3 类场景（plan.md §4.1）每类 ≥ 5 个目标，每个目标 5 次重复，共 ≥ 75 次单帧检测。

## 5. 不变量

- F2 节点崩溃 / VLM 超时 → **不能**让飞行链路掉出 OFFBOARD（即不能阻塞 setpoint_bridge 的 setpoint 流）。设计上 F2 与 setpoint_bridge 是异步的，本来就满足，但要测一次拔网线场景。
- 输出 frame 必须是 `map`，不能是 `odom` 或 `base_link`。
- API key 必须支持环境变量注入，**不能**硬编码进 launch（当前 launch 已硬编码，F2 完成时一并清掉）。

## 6. 风险

| 风险 | 影响 | 兜底 |
|---|---|---|
| Qwen-VL grounding 输出格式变化 | 解析失败 | `_parse_bbox` 多正则兜底 + 兼容老格式 |
| 坐标归一化基数不确定（1000 vs 像素） | 框落到错位置 | 第一次跑必须看调试图像目视确认；写到 plan.md |
| 框内深度全无效（透明/反射/远处） | 没有 3D 点 | 返回 None，上层重试或扩大邻域 |
| API 限流 | 节点饿死 | 限制最大并发 1，超时直接 abort 当帧 |
| VLM 幻觉（图里没有目标也给框） | 飞向虚假目标 | 用 confidence 阈值；目标丢失多次进入 fallback（plan.md §3.2 写法） |

## 7. 与现状的差异（Migration）

`vlm_navigation.py` 当前角色 = "**Detector + 坐标转换 + Goal 发布器**" 三合一。F2 完成后拆为：
- `detector_node.py`（新）+ `detection/qwen_vl_grounding.py`（新）→ 接 ABC
- `vlm_navigation.py` 缩减为"任务编排"角色（订阅 instruction、调度 detector、决定何时进入起飞 / 降落），后续可被更通用的状态机取代

迁移期 `vlm_navigation.py` 与 `detector_node.py` 共存，old launch 仍能跑；新 launch (`vlm_navigation_v2.launch`) 跑新链路。等 F4 完成后整体替换。
