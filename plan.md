# 基于视觉语言模型的无人机语言交互与自主导航（仿真优先 + EGO-Planner）计划书

> 适用范围：硕士/毕设项目计划（20 周）。
> 
> 核心原则：**先闭环、后增强；先仿真、后实机；先可复现、后追性能**。

---

## 1. 项目概述

### 1.1 题目
基于视觉语言模型的无人机语言交互与自主导航系统设计与实现（仿真优先）。

### 1.2 总目标
实现无人机在 ROS + Gazebo/PX4 SITL 仿真中：
1) 接收自然语言指令；
2) 解析为结构化任务（JSON）；
3) 通过视觉/深度感知定位目标；
4) 基于 **EGO-Planner** 生成动力学可行轨迹并避障；
5) 通过 MAVROS/Offboard 控制 PX4 执行并完成任务。

### 1.3 研究问题与工程闭环
- **语义到几何**：将"救援目标/物资箱/障碍"等语义概念稳定转换为可规划的 3D 坐标与约束。
- **实时规划**：在部分未知/动态障碍场景中进行局部重规划，输出可飞轨迹。
- **系统集成**：ROS 多节点低延迟通信、可复现评测与记录。

---

## 2. 总体技术路线（与开题对齐）

### 2.1 系统分层
1. **自然语言理解层（VLM）**
   - 输入：中文自然语言指令
   - 输出：VLM 直接输出目标 bbox / 像素坐标

2. **视觉感知层（VLM Grounding + 深度融合）**
   - 输入：RGB + Depth + 目标描述
   - 输出：目标 3D 点（map 坐标系）+ 调试图像

3. **轨迹规划层（EGO-Planner）**
   - 输入：当前状态、深度点云/ESDF、目标点
   - 输出：动力学可行轨迹（B-spline）

4. **控制执行层（PX4 SITL + MAVROS）**
   - 输入：期望位姿/轨迹设定点（Offboard）
   - 输出：无人机执行 + 状态反馈

### 2.2 "最小可交付版本（MVP）"定义 ✅ 已达成
- ✅ 在 Gazebo 仿真场景中，通过一句自然语言指令触发任务
- ✅ 系统完成"目标定位 → 规划 → 飞行 → 到达目标半径内悬停"闭环
- ✅ 有一键复现实验脚本与日志

---

## 3. 进度跟踪

### ✅ 第 1 阶段：环境与基础飞行闭环（第 1-4 周）— 已完成

**产出**
- ROS + Gazebo + PX4 SITL + MAVROS 完整跑通
- Offboard 起飞/悬停/降落稳定运行
- RGB/Depth 相机话题正常
- 一键启动脚本 `scripts_start_px4_sitl_gazebo.sh`

### ✅ 第 2 阶段：VLM 检测与 3D 定位（第 5-8 周）— 已完成

**产出**
- Qwen-VL (DashScope API) visual grounding：bbox 解析 + 深度 back-project
- TF 链打通：`camera_link_optical` → `map`
- `detector_node.py`：模块化检测 ROS 节点
- 关键 Bug 修复：
  - TF 时间戳修正（使用图像拍摄时刻而非当前时刻，消除数米级偏差）
  - bbox 归一化条件修正（Qwen-VL [0,1000] → 像素坐标）
  - 深度融合优化（中心 25% 区域取深度，避免背景污染）

### ✅ 第 3 阶段：EGO-Planner 规划与控制集成（第 9-12 周）— 已完成

**产出**
- `ego_adapter.py`：EGO-Planner C++ 节点与 Python 的桥接
- `planner_node.py`：通用规划器 ROS 节点（importlib 动态加载）
- `trajectory_follower.py`：30Hz 轨迹插值 → setpoint 流
- `vlm_navigation_full.launch`：端到端完整链路
- 鲁棒性加固（2026-06-17）：
  - 防崩溃守卫（cam_info None 检查、plan() try-except）
  - 回调不阻塞（规划移至后台线程）
  - 并发保护（阻止重复规划触发）
  - TF 健康检查增强（base_link→camera_link）

### 🔜 第 4 阶段：状态控制与持续搜索（第 13-16 周）— 当前

**目标 1：状态机控制**

当前系统是"单次检测→单次飞行"的线性模式。需要实现多状态 FSM：

```
IDLE → TAKEOFF → SEARCHING → TARGET_FOUND → APPROACHING → HOVER_AT_TARGET
                   ↑                                    |
                   └────────── TARGET_LOST ←────────────┘
```

关键行为：
- **SEARCHING**: 无目标时自主搜索（旋转扫描 / 航点遍历 / 螺旋搜索）
- **TARGET_FOUND → APPROACHING**: 检测到目标后规划并靠近
- **TARGET_LOST → SEARCHING**: 飞行中丢失目标时恢复搜索
- 状态迁移由 `/uav/state_cmd` 和检测结果共同驱动

**目标 2：VLM Prompt 对比实验**

比较两种提示策略对检测精度的影响：

| 方案 A | 方案 B |
|--------|--------|
| "请框出图中的{target}" | "请返回图中{target}的像素坐标" |
| 直接返回 bbox 格式 | 返回 (x, y) 像素坐标值 |
| 当前默认方案 | 待测试 |

评测维度：
- 检测成功率（是否给出有效坐标）
- 2D 中心偏差（像素）：`||center_pred - center_gt||`
- 3D 定位误差（米）：`||pos_pred - pos_gt||`（如果能获取真值）
- API 响应延迟
- 输出格式解析成功率

实验设计：
- 固定场景集：3-5 个不同距离/角度的目标位置
- 每种目标每种 prompt 重复 5 次
- 输出对比表格 + 统计检验

### 🔜 第 5 阶段：系统实验、对比与论文（第 17-20 周）

- 批量跑场景集，自动汇总指标
- 对比基线：无语言（直接给坐标）、无重规划（只规划一次）
- 论文撰写与图表生成

---

## 4. 关键模块设计（已实现部分）

### 4.1 VLM 检测 → 3D 定位

数据流：
```
RGB + Depth (同步帧)
  → Qwen-VL API (base64 JPEG, grounding prompt)
  → bbox 解析 (x1,y1,x2,y2, 归一化→像素)
  → 深度融合 (bbox 中心 25% 区域中值)
  → 像素 back-project (相机内参 K)
  → TF 变换 camera_optical → map (使用图像时间戳)
  → /uav/target_world (PointStamped, latched)
```

### 4.2 EGO-Planner 集成

```
/uav/target_world + /mavros/local_position/pose
  → planner_node (后台线程)
  → ego_adapter.plan() → /move_base_simple/goal
  → EGO-Planner C++ 节点 → /planning/bspline
  → B-spline → Trajectory 数据类
  → /uav/trajectory (Path, latched)
  → trajectory_follower (30Hz 插值)
  → /uav/goal_pose
  → setpoint_bridge (FSM, MAVROS Offboard)
```

### 4.3 模块契约 (ABC + importlib)

```python
# 更换 VLM 实现：改 launch 文件一个参数
<param name="detector_class" value="qwen_vl_grounding.QwenVLGroundingDetector"/>

# 更换规划器：改 launch 文件一个参数
<param name="planner_class" value="planning.ego_adapter.EgoPlannerAdapter"/>
```

---

## 5. 数据、场景与评测设计

### 5.1 仿真场景集
- **当前场景**: `search_rescue.world` — 起降台 + 4 个红色支架障碍物
- **待扩展**: 多障碍/干扰物场景、动态障碍场景

### 5.2 指标

**感知指标**
- 检测成功率（%）
- 3D 定位误差（m）：`||p_est - p_gt||`

**规划与控制指标**
- 任务成功率（%）：到达目标半径内
- 轨迹发布延迟（s）

**Prompt 实验指标（新增）**
- 坐标解析成功率
- 2D 中心偏差（像素）
- API 延迟

---

## 6. 风险与应对

| 风险 | 影响 | 优先级 | 应对/兜底 |
|---|---|---|---|
| VLM/检测不稳定 | 目标点抖动 | 高 | Prompt 优化实验；多帧滤波；目标丢失进入搜索 |
| 持续搜索效率低 | 任务超时 | 中 | 先实现简单旋转扫描，再扩展航点遍历 |
| Prompt 实验无显著差异 | 论文对比缺失 | 低 | 可增加第三种 prompt 或扩展到不同 VLM 模型对比 |

---

## 7. 近期行动清单

1. 实现状态机控制节点（独立 ROS 节点或集成到 planner_node）
2. 实现 SEARCHING 状态：旋转扫描策略
3. 设计 Prompt 对比实验脚本，固定场景 + 重复测试
4. 收集实验数据：坐标偏差、定位误差、延迟
5. 更新 CLAUDE.md 和文档
