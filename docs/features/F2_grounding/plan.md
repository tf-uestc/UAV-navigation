# F2 — Plan: VLM Grounding 实现方案

## 1. 总览

新增三个文件 + 拆出一个老文件：

| 文件 | 角色 |
|---|---|
| `detection/base.py` | `Detector` ABC + `Detection` dataclass |
| `detection/qwen_vl_grounding.py` | Qwen2-VL grounding 实现（baseline）|
| `detector_node.py` | 通用检测节点入口，按 `~detector_class` 加载实现 |
| `vlm_navigation.py` (改) | 移除 VLM 调用与坐标投影，只保留任务编排 |

老的 `vlm_navigation.py` 内部的 `VLMApiClient` 和 `DepthProjector` 大部分可以**搬过去**，不是从头写。

## 2. Detector ABC 与 Detection 数据结构

```python
# detection/base.py
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np
from sensor_msgs.msg import CameraInfo

@dataclass
class Detection:
    bbox: Tuple[int, int, int, int]   # x1, y1, x2, y2 (像素)
    center: Tuple[int, int]
    depth_m: float
    confidence: float
    raw: dict = field(default_factory=dict)
    target_text: str = ""

class Detector(ABC):
    @abstractmethod
    def detect(self,
               rgb: np.ndarray,
               depth: np.ndarray,
               target_text: str,
               cam_info: CameraInfo) -> Optional[Detection]:
        ...

    def healthcheck(self) -> bool:
        """default: always healthy. 子类可覆盖以测 API 连通性。"""
        return True
```

## 3. QwenVLGroundingDetector 实现要点

### 3.1 Prompt

```python
DEFAULT_PROMPT = "请框出图中的{target}。只输出一个框,不要解释。"
```

### 3.2 API 调用

继续走 DashScope 的 OpenAI 兼容接口（已可用），不变：

```python
url = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
model = "qwen-vl-max"   # 或 qwen2-vl-72b-instruct (grounding 更稳)
```

> Qwen2.5-VL 系列的 grounding 训练数据更多，效果更好；先用 qwen-vl-max 跑通，再切换 qwen2-vl-7b-instruct 或 qwen-vl-plus 做对比。

### 3.3 输出解析（核心难点）

按以下顺序尝试，命中即返回：

```python
PATTERNS = [
    # Qwen2-VL 格式
    r"<\|box_start\|>\((\d+),(\d+)\),\((\d+),(\d+)\)<\|box_end\|>",
    # Qwen-VL 经典格式
    r"<box>\s*\((\d+),(\d+)\)\s*,\s*\((\d+),(\d+)\)\s*</box>",
    # 兜底:JSON / 纯文本数组
    r"\[?\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]?",
]
```

**坐标尺度**：Qwen 系列约定输出归一化到 `[0, 1000]`。检测规则：

```python
if max(x1, y1, x2, y2) <= 1000 and (img_w > 1000 or img_h > 1000 or
                                     bbox_area_ratio < 0.5):
    # 视为归一化,缩放回像素
    x1 = int(x1 * img_w / 1000); ...
```

⚠️ 这条规则在 848×480 图像上有边角情况（1000 内 vs 800 内）。**第一次跑必须看 `/uav/target_debug` 把框画出来肉眼确认**。如果发现误判，最稳的办法是按模型版本写死 scale（qwen-vl-max → 像素，qwen2-vl → 1000 归一化）。

### 3.4 框内深度融合

不再用单点 3×3 邻域，改成框内中位数：

```python
def depth_in_bbox(depth, x1, y1, x2, y2) -> Optional[float]:
    patch = depth[y1:y2, x1:x2]
    valid = patch[(~np.isnan(patch)) & (patch > 0) & np.isfinite(patch)]
    if valid.size < 0.3 * patch.size:    # 有效像素 < 30% 就放弃
        return None
    return float(np.median(valid))
```

`Detection.confidence` 用"框内有效深度比"作为 proxy（后续可换 VLM 自带置信度）。

### 3.5 像素 → 相机系（依赖 F3 的 TF 链做相机系→世界系）

F2 内部只负责到**相机光学系**：

```python
def pixel_to_camera_optical(u, v, depth, fx, fy, cx, cy):
    # ROS optical frame: x 朝右, y 朝下, z 朝前
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return x, y, z
```

世界系转换在 `detector_node.py` 里调用 `tf2_buffer.transform()`（F3 的产物）。

## 4. detector_node 节点

```python
# detector_node.py 骨架
class DetectorNode:
    def __init__(self):
        cls_path = rospy.get_param("~detector_class",
                                    "detection.qwen_vl_grounding.QwenVLGroundingDetector")
        self.detector = self._load_class(cls_path)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        # ... 订阅图像 / instruction, 发布 target_world / target_debug
```

`_load_class` 用 `importlib.import_module + getattr`，把构造参数从 `~detector_args` (字典) 注入。

## 5. 调试图像

在 `/uav/target_debug` 上叠加：
- 绿色框（bbox）
- 红色十字（中心）
- 文本：`target | depth=2.34m | conf=0.85 | t=1.20s`

调试图像是论文里截图、答辩演示、跑回归测试的最直接产物，**不要省**。

## 6. 配置与 launch

新建 `vlm_navigation_v2.launch`，与老 launch 并存：

```xml
<node pkg="uav_vln_bringup" type="detector_node.py" name="detector" output="screen">
  <param name="detector_class"
         value="detection.qwen_vl_grounding.QwenVLGroundingDetector"/>
  <param name="vlm_provider" value="qwen"/>
  <param name="vlm_model" value="qwen-vl-max"/>
  <!-- 不再硬编码 key:从环境变量读 -->
  <param name="vlm_api_key" value="$(env VLM_API_KEY)"/>
  <param name="confidence_threshold" value="0.3"/>
  <param name="trigger_rate" value="0.2"/>
</node>
```

启动时：

```bash
export VLM_API_KEY=sk-xxx
roslaunch uav_vln_bringup vlm_navigation_v2.launch
```

## 7. 与现状的回退路径

如果 F2 出问题，老链路 `roslaunch uav_vln_bringup vlm_navigation.launch` 继续可用。等 F4 完成、整体迁移成功后才删除老 launch 与 `vlm_navigation.py` 里的 VLM/Projector 代码。

## 8. 取舍与备选

| 决策 | 选择 | 备选 | 理由 |
|---|---|---|---|
| API 类型 | DashScope OpenAI 兼容 | 原生 DashScope SDK | 现有代码已用，迁移成本零 |
| 模型 | qwen-vl-max → 后续切 qwen2-vl | gpt-4o (闭源海外) | 中文 prompt + 国内可用性 |
| 检测器抽象层 | Detector ABC | 直接 ROS 节点 + topic | 单测友好，论文写"模块化"有据可依 |
| 深度融合 | 框内中位数 | 框中心 + 邻域 | 中位数对外点鲁棒得多 |
| 多帧滤波 | **不在 F2 内** | EMA / Kalman | 留作 future,先看 baseline 抖动有多大再说 |

## 9. 风险（细化 spec.md §6）

- **"还要一段时间才能验"的风险**：F2 看似"换个 prompt + 换个解析"，但坐标归一化 + bbox→3D 的稳定性需要在 ≥5 个场景反复验，留 1 周缓冲。
- **API 计费**：grounding 模式调用频率别太高（默认 0.2 Hz = 5s 一次足够）。否则一晚上几十块。
- **Qwen 升级 / 降级**：DashScope 偶尔下线模型；写代码时把 `model` 做参数，别 hardcode。
