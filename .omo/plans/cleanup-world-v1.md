# 项目清理 + 新世界模型 + 去 v1

## TL;DR
> **Quick Summary**: 清理项目中 dummy/test 代码、移除 v1 旧架构、创建新 Gazebo 世界模型（黄色起降平台 + 红色支架障碍物 + 提亮光照），并自动化 .env 加载。
>
> **Deliverables**:
> - .env 自动加载方案
> - 删除 `dummy_detector.py` 及相关引用
> - 新 `simulation/px4_gazebo_classic/worlds/search_rescue.world` 世界文件
> - 移除 `vlm_navigation.py` / `vlm_navigation.launch` v1 代码
>
> **Estimated Effort**: Medium
> **Parallel Execution**: YES - 4 waves
> **Critical Path**: Task 1 (.env) → Task 4/5 (模型) → Task 6 (world) → Task 7 (编译)

---

## Context

### Original Request
1. `.env` 的 key 每次要手动 export，需要自动化 — **放在 `scripts_start_px4_sitl_gazebo.sh` 中加载**
2. 去掉 `dummy_detector.py`，代码太乱
3. 新建 Gazebo 世界模型（黄色起降平台 + 红杆支架障碍物 + 光照调亮）
4. 去掉 v1 版本，**物理删除** `vlm_navigation.py` 和 `vlm_navigation.launch`，`vlm_navigation_v2.launch` → `vlm_navigation.launch`（去 _v2 后缀）

### 当前状态
- `detector_node.py`: 已修 latch + vlm_model 传参，QwenVL 正常工作
- `qwen_vl_grounding.py`: 日志已改 rospy，qwen3.6-plus 调用成功
- `vlm_navigation_v2.launch`: arg 传参已修，model=qwen3.6-plus
- `.env`: 已在 .gitignore，含 VLM_API_KEY
- `dummy_detector.py`: 测试用，保留在 CMakeLists.txt + 作为默认 detector
- `vlm_navigation.py`: v1 旧版，仍在 CMakeLists.txt install
- `vlm_navigation.launch`: v1 launch，仍可运行
- simulation 目录: 仅有 `simulation/px4_gazebo_classic/` 框架

---

## Work Objectives

### Core Objective
清理测试代码和旧架构，创建完整的仿真世界模型，自动化环境配置。

### Concrete Deliverables
- `scripts_start_px4_sitl_gazebo.sh` 添加 `.env` 加载逻辑
- 物理删除 `scripts/dummy_detector.py`
- 物理删除 `scripts/vlm_navigation.py`
- 物理删除 `launch/vlm_navigation.launch`（v1 版）
- 重命名 `launch/vlm_navigation_v2.launch` → `launch/vlm_navigation.launch`
- 更新 `CMakeLists.txt` 移除 vlm_navigation.py + 更新 launch 引用
- 更新 `scripts_demo_all.sh` 指向新 launch 名
- 新建 `simulation/px4_gazebo_classic/worlds/search_rescue.world`
- 新建 `simulation/px4_gazebo_classic/models/landing_pad/`
- 新建 `simulation/px4_gazebo_classic/models/red_tripod/`
- 删除或标记废弃 `vlm_navigation.py` / `vlm_navigation.launch`

### Definition of Done
- [ ] `source scripts_start_px4_sitl_gazebo.sh` 自动加载 VLM_API_KEY
- [ ] `dummy_detector.py` 和 `vlm_navigation.py` 物理删除
- [ ] `vlm_navigation_v2.launch` → `vlm_navigation.launch`，无 _v2 后缀残留
- [ ] `roslaunch uav_vln_bringup vlm_navigation.launch` 默认使用 QwenVL
- [ ] Gazebo 加载 `search_rescue.world` 显示黄色平台 + 两红色障碍物 + 亮场景
- [ ] `CMakeLists.txt` 无 vlm_navigation.py 引用
- [ ] 所有改动通过 `catkin_make` 编译

### Must Have
- .env 在 `scripts_start_px4_sitl_gazebo.sh` 中自动加载
- v1 代码物理删除，v2 去后缀
- 世界模型完整可运行
- v2（新 vlm_navigation.launch）为唯一启动方式

### Must NOT Have
- `dummy_detector.py` 任何残留
- `vlm_navigation.py` / 旧 `vlm_navigation.launch` 任何残留
- `_v2` 后缀残留
- 世界文件里硬编码无人机模型（外部 spawn）
- 硬编码 API key

---

## Verification Strategy

- **Test Decision**: 无自动化测试框架 — Agent-Executed QA 为主
- **Framework**: bash 命令验证 + Gazebo 加载验证

### QA Policy
- CLI/文件: Bash 验证文件存在/不存在
- Gazebo: 手动加载 world 文件确认视觉效果
- ROS: roslaunch 确认节点启动无报错

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (清理 — 全部并行):
├── Task 1: .env 自动加载 — 在 scripts_start_px4_sitl_gazebo.sh 中
├── Task 2: 物理删除 dummy_detector + 更新默认值
└── Task 3: 物理删除 v1 + 重命名 v2 → v1

Wave 2 (世界模型 — 并行):
├── Task 4: 创建 landing_pad 模型
└── Task 5: 创建 red_tripod 障碍物模型

Wave 3 (世界文件 — 依赖 Wave 2):
└── Task 6: 创建 search_rescue.world 主文件

Wave 4 (验证):
├── Task 7: 编译验证
└── Task 8: roslaunch + Gazebo 验证
```

---

## TODOs

- [ ] 1. .env 自动加载 — 在 `scripts_start_px4_sitl_gazebo.sh` 中加载

  **What to do**:
  - 在 `scripts_start_px4_sitl_gazebo.sh` 中，`# ========== 加载 ROS ==========` 之前添加：
    ```bash
    # ========== 加载环境变量 ==========
    if [ -f "$ROOT_DIR/.env" ]; then
        export $(grep -v '^\s*#' "$ROOT_DIR/.env" | xargs)
        echo "✓ .env loaded"
    fi
    ```

  **Must NOT do**:
  - 不要把 .env 内容直接写进脚本
  - 不要把 API key 硬编码

  **Parallelization**: Wave 1 — 与 Task 2/3 并行

  **QA Scenarios**:
  ```
  Scenario: source 脚本后 VLM_API_KEY 自动生效
    Tool: Bash
    Steps:
      1. unset VLM_API_KEY
      2. bash -c 'source scripts_start_px4_sitl_gazebo.sh 2>/dev/null; echo $VLM_API_KEY' | head -20
    Expected Result: 输出包含 sk-faeac5f4...（非空字符串）和 ✓ .env loaded
    Evidence: .omo/evidence/task-1-env.txt
  ```

- [ ] 2. 物理删除 dummy_detector + 更新默认值

  **What to do**:
  - 物理删除 `ws/src/uav_vln_bringup/scripts/dummy_detector.py`
  - 更新 `launch/vlm_navigation_v2.launch`（即将重命名）默认值：
    `<arg name="detector_class" default="qwen_vl_grounding.QwenVLGroundingDetector" />`
  - 更新 `launch/vlm_navigation_v2.launch` 注释，移除 dummy 相关说明
  - `detector_node.py` 默认值改为 `qwen_vl_grounding.QwenVLGroundingDetector`

  **Must NOT do**:
  - 不要删 `base_detector.py` 或 `qwen_vl_grounding.py`

  **Parallelization**: Wave 1 — 与 Task 1/3 并行

  **QA Scenarios**:
  ```
  Scenario: dummy 文件已删除
    Tool: Bash
    Steps:
      1. ls ws/src/uav_vln_bringup/scripts/dummy_detector.py
    Expected Result: No such file or directory
    Evidence: .omo/evidence/task-2-dummy.txt

  Scenario: launch 默认使用 QwenVL
    Tool: Bash (grep)
    Steps:
      1. grep "detector_class" launch/vlm_navigation.launch
    Expected Result: default="qwen_vl_grounding.QwenVLGroundingDetector"
    Evidence: .omo/evidence/task-2-launch.txt
  ```

- [ ] 3. 物理删除 v1 + 重命名 v2 → v1

  **What to do**:
  - 物理删除 `scripts/vlm_navigation.py`
  - 物理删除 `launch/vlm_navigation.launch`（v1 旧版）
  - 重命名 `launch/vlm_navigation_v2.launch` → `launch/vlm_navigation.launch`
  - 更新 `CMakeLists.txt`：删除 `scripts/vlm_navigation.py` 行
  - 更新 `scripts_demo_all.sh`：`vlm_navigation_v2.launch` → `vlm_navigation.launch`
  - 在新 `vlm_navigation.launch` 内注释中移除 _v2 相关描述

  **Must NOT do**:
  - 不要删除 `setpoint_bridge.py` 或 `takeoff_land.py`
  - 不要删除 `launch/mavros_px4_sitl.launch`
  - 不要留下 _v2 后缀的任何引用

  **Parallelization**: Wave 1 — 与 Task 1/2 并行

  **QA Scenarios**:
  ```
  Scenario: v1 文件已物理删除
    Tool: Bash
    Steps:
      1. ls ws/src/uav_vln_bringup/scripts/vlm_navigation.py
      2. ls ws/src/uav_vln_bringup/launch/vlm_navigation_v2.launch
    Expected Result: 两者均 No such file
    Evidence: .omo/evidence/task-3-deleted.txt

  Scenario: 新 vlm_navigation.launch 存在
    Tool: Bash
    Steps:
      1. ls ws/src/uav_vln_bringup/launch/vlm_navigation.launch
    Expected Result: 文件存在（由 _v2 重命名而来）
    Evidence: .omo/evidence/task-3-renamed.txt

  Scenario: _v2 后缀无残留
    Tool: Bash (grep)
    Steps:
      1. grep -r "_v2" ws/src/uav_vln_bringup/launch/ scripts_demo_all.sh CMakeLists.txt 2>/dev/null
    Expected Result: 无匹配（或仅在注释说明中）
    Evidence: .omo/evidence/task-3-nov2.txt
  ```

- [ ] 4. 创建 landing_pad 黄色起降平台模型

  **What to do**:
  - 创建目录 `simulation/px4_gazebo_classic/models/landing_pad/`
  - 创建 `model.sdf`：圆柱体 r=1.3m, h=0.02m, 位置 (0,0,0.01)
    - 材质：哑光淡黄色 RGB(1, 0.92, 0.15), `<ambient>` 匹配
    - static=true, 带碰撞 `<collision>`
  - 顶面白色十字标记：两个薄长方体（0.001m 厚），十字交叉，贴平台顶面 z=0.021
    - 白色 RGB(1,1,1)
  - 创建 `model.config`：name=landing_pad

  **Must NOT do**:
  - 不要把平台写进 world 文件（保持模型独立复用）

  **Parallelization**: Wave 2 — 与 Task 5 并行，Task 6 依赖

  **QA Scenarios**:
  ```
  Scenario: 模型文件完整
    Tool: Bash
    Steps:
      1. ls simulation/px4_gazebo_classic/models/landing_pad/model.sdf
      2. ls simulation/px4_gazebo_classic/models/landing_pad/model.config
    Expected Result: 两个文件均存在
    Evidence: .omo/evidence/task-4-model.txt
  ```

- [ ] 5. 创建 red_tripod 红色支架障碍物模型

  **What to do**:
  - 创建目录 `simulation/px4_gazebo_classic/models/red_tripod/`
  - 创建 `model.sdf`：
    - **主体**: 低暗红长圆柱 RGB(0.72, 0.18, 0.12)，长度 0.9m，半径 0.05m，沿 Z 轴（或水平放置）
    - **三根黑色细支架**: 半径 0.03m，三角分叉落地，固定在主杆尾部
      - 支架1: 从主杆尾部向地面方向倾斜
      - 支架2/3: 向两侧分叉
      - 黑色 RGB(0.15, 0.15, 0.15)
    - 全部 link 绑定为一个 model，static=true，带碰撞
    - 整体风格：哑光低饱和，无亮艳色
  - 创建 `model.config`：name=red_tripod

  **Must NOT do**:
  - 不要用立方体（box），只用圆柱体（cylinder）
  - 不要使用高饱和鲜艳色

  **Parallelization**: Wave 2 — 与 Task 4 并行，Task 6 依赖

  **QA Scenarios**:
  ```
  Scenario: 模型文件完整
    Tool: Bash
    Steps:
      1. ls simulation/px4_gazebo_classic/models/red_tripod/model.sdf
      2. ls simulation/px4_gazebo_classic/models/red_tripod/model.config
    Expected Result: 两个文件均存在
    Evidence: .omo/evidence/task-5-model.txt
  ```

- [ ] 6. 创建 search_rescue.world 世界文件

  **What to do**:
  - 创建 `simulation/px4_gazebo_classic/worlds/search_rescue.world`
  - SDF 1.7 格式，ODE 物理引擎
  - **光照配置**：
    ```xml
    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <sky>
        <sun>
          <diffuse>0.92 0.92 0.92 1</diffuse>
          <direction>0.5 0.3 -0.8</direction>
          <cast_shadows>true</cast_shadows>
        </sun>
      </sky>
    </scene>
    ```
  - **点光源**: 位置 (0, 0, 9), 暖白色 diffuse(0.7, 0.65, 0.55), 衰减
  - **地面**：深灰色哑光平面 `<plane>` color(0.2, 0.2, 0.2)
  - **include landing_pad 模型**: pose(-1.01, -0.98, 0.01, 0, 0, 0)
  - **include red_tripod 模型 A**: pose(0.25, 0.22, 0.12, 0, 0, 0)
  - **include red_tripod 模型 B**: pose(0.42, -2.15, 0.12, 0, 0, 0)
  - **相机初始视角**: `<gui>` 中设置 `<camera>` 俯视斜拍，pose 能完整包含平台和两障碍物，如 pose(0, -1, 5, 0, 0.6, 0)
  - **无人机**: 不在 world 中写 spawn（由 launch 外部注入）
  - 代码附注释说明各区域

  **Must NOT do**:
  - 不在 world 内写 `<model name='iris'>` 无人机
  - 不引用不存在的模型路径

  **Parallelization**: Wave 2 — 依赖 Task 4/5

  **QA Scenarios**:
  ```
  Scenario: Gazebo 加载 world 成功
    Tool: Bash
    Steps:
      1. gazebo --verbose simulation/px4_gazebo_classic/worlds/search_rescue.world &
      2. sleep 5
      3. kill %1
    Expected Result: Gazebo 无报错退出，无模型加载失败
    Evidence: .omo/evidence/task-6-gazebo.txt
  ```

- [ ] 7. 编译验证

  **What to do**:
  - `cd ~/毕业设计/ws && catkin_make`
  - 确认编译无错误
  - 确认 dummy_detector.py 确实不存在
  - 确认 vlm_navigation.py 不在 install target

  **Parallelization**: Wave 3 — 与 Task 8 顺序（先编译后 launch）

  **QA Scenarios**:
  ```
  Scenario: catkin_make 通过
    Tool: Bash
    Steps:
      1. cd ws && catkin_make 2>&1 | tail -20
    Expected Result: 无 "Error" 字样，返回码 0
    Evidence: .omo/evidence/task-7-build.txt
  ```

- [ ] 8. ROS launch 验证

  **What to do**:
  - 确保 `.env` 已加载
  - 启动 `roslaunch uav_vln_bringup vlm_navigation_v2.launch`
  - 确认日志显示 `Loaded detector: qwen_vl_grounding.QwenVLGroundingDetector`
  - 发一条 instruction 确认检测链路通

  **Parallelization**: Wave 3 — 依赖 Task 7

  **QA Scenarios**:
  ```
  Scenario: 默认启动使用 QwenVL
    Tool: Bash (roslaunch)
    Steps:
      1. roslaunch uav_vln_bringup vlm_navigation_v2.launch &
      2. sleep 10
      3. grep "Loaded detector" ~/.ros/log/latest/detector_node*.log
    Expected Result: 包含 "qwen_vl_grounding.QwenVLGroundingDetector"
    Evidence: .omo/evidence/task-8-launch.txt
  ```

---

## Final Verification Wave

- [ ] F1. 编译验证 — `catkin_make` 成功
- [ ] F2. 文件清理验证 — `dummy_detector.py` 不存在, `vlm_navigation.py` 不存在, 旧 `vlm_navigation.launch` 不存在, `_v2` 后缀无残留
- [ ] F3. 世界加载验证 — Gazebo 加载 search_rescue.world 成功
- [ ] F4. 启动验证 — `roslaunch uav_vln_bringup vlm_navigation.launch` 默认 QwenVL 启动成功

---

## Commit Strategy

- **Wave 1**: `chore: remove dummy detector and v1 code, auto-load .env, rename v2 to v1`
  - Files: scripts_start_px4_sitl_gazebo.sh, scripts/dummy_detector.py (删), scripts/vlm_navigation.py (删), launch/vlm_navigation.launch (删旧+重命名v2), CMakeLists.txt, scripts_demo_all.sh
- **Wave 2**: `feat: add search_rescue world with landing pad and tripod obstacles`
  - Files: simulation/px4_gazebo_classic/worlds/search_rescue.world, simulation/px4_gazebo_classic/models/landing_pad/*, simulation/px4_gazebo_classic/models/red_tripod/*

---

## Success Criteria

### Verification Commands
```bash
# .env 加载验证
bash -c 'source scripts_start_px4_sitl_gazebo.sh 2>/dev/null; echo $VLM_API_KEY' | head -5

# 文件清理验证
ls ws/src/uav_vln_bringup/scripts/dummy_detector.py    # Expected: No such file
ls ws/src/uav_vln_bringup/scripts/vlm_navigation.py    # Expected: No such file
ls ws/src/uav_vln_bringup/launch/vlm_navigation_v2.launch  # Expected: No such file
ls ws/src/uav_vln_bringup/launch/vlm_navigation.launch # Expected: 文件存在

# 编译验证
cd ws && catkin_make                                     # Expected: 编译成功

# 世界加载验证
gazebo simulation/px4_gazebo_classic/worlds/search_rescue.world  # Expected: 加载成功

# 启动验证
roslaunch uav_vln_bringup vlm_navigation.launch  # Expected: Loaded detector: qwen_vl_grounding
```

### Final Checklist
- [ ] `scripts_start_px4_sitl_gazebo.sh` 自动加载 .env
- [ ] `dummy_detector.py` 物理删除
- [ ] `vlm_navigation.py` / 旧 `vlm_navigation.launch` 物理删除
- [ ] `vlm_navigation_v2.launch` → `vlm_navigation.launch`
- [ ] 无 _v2 后缀残留
- [ ] 新世界可加载
- [ ] `catkin_make` 通过
