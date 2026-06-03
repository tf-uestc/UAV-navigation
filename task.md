# 本地 Agent 执行任务清单

> 任务范围：完成需求 1（.env 加 export 前缀）、2（删除 dummy_detector）、4（新世界文件）、5（删除 v1）、6（默认 model 改 qwen3.6-plus）。
> 项目根目录：`UAV-navigation-main/`（以下路径都相对于此）
> 执行约束：每个任务都是原子操作，按顺序执行。**不要跳步**。

---

## 任务 1：`.env` / `.env.example` 加 export 前缀

**背景**：
- `docs/.env.example` 是模板（占位符 `sk-xxx`，已提交到 git）
- 项目根目录的 `.env` 是用户本地真 key 文件（`.gitignore` 已忽略）
- 用户每次都要手动 `export VLM_API_KEY=...`，根因：`.env` 里写的是 `VLM_API_KEY=sk-xxx`（没 `export` 前缀），`source .env` 只能让变量在当前 shell 可见，**不会进环境变量**，所以 launch 文件的 `$(env VLM_API_KEY)` 读不到。
- 加上 `export` 前缀后，`source .env` 就能自动把变量 export 到环境变量，后续 `roslaunch` 直接能读到。
- **本任务只做这件事**。自动化 launch source .env 留到后续阶段（无人机起飞 + 路径规划模块完成后再做）。

### 1.1 修改 `docs/.env.example`

打开 `docs/.env.example`，找到最后一行：
```
VLM_API_KEY=sk-xxx
```
改为：
```
export VLM_API_KEY=sk-xxx
```

### 1.2 修改本地 `.env`（如果存在）

```bash
# 先检查 .env 是否存在
ls -la .env 2>/dev/null
```

**如果 `.env` 存在**：打开它，给每一行 `VLM_API_KEY=...`（或其他 `KEY=VALUE` 行）前面加 `export ` 前缀。例如：
```
VLM_API_KEY=sk-faeac5f4abc...
```
改为：
```
export VLM_API_KEY=sk-faeac5f4abc...
```
**保留真实 key 不变**，只加 `export ` 前缀。

**如果 `.env` 不存在**：跳过 1.2，不需要做任何事。

### 1.3 验证

```bash
# .env.example 末行有 export
tail -1 docs/.env.example
# 期望: export VLM_API_KEY=sk-xxx

# 本地 .env (如果存在) 有 export
[[ -f .env ]] && grep "^export VLM_API_KEY" .env && echo "✓ .env has export prefix"

# 干跑测试 source 流程
[[ -f .env ]] && bash -c 'unset VLM_API_KEY; source .env; [[ -n "$VLM_API_KEY" ]] && echo "✓ VLM_API_KEY exported after source"'
```

**使用方式（用户每次启动 ROS 节点前）**：
```bash
source .env
roslaunch uav_vln_bringup vlm_navigation.launch
```

---

## 任务 2：删除 `dummy_detector.py` 及所有引用

**目的**：清理骨架测试代码。

### 2.1 删除文件

```bash
rm -f ws/src/uav_vln_bringup/scripts/dummy_detector.py
```

### 2.2 修改 `ws/src/uav_vln_bringup/scripts/detector_node.py`

找到这一行（约第 73 行）：
```python
cls_path = rospy.get_param("~detector_class", "dummy_detector.DummyDetector")
```
改为：
```python
cls_path = rospy.get_param("~detector_class", "qwen_vl_grounding.QwenVLGroundingDetector")
```

### 2.3 修改 `ws/src/uav_vln_bringup/launch/vlm_navigation_v2.launch`

**注意**：本地这个文件还存在（任务 5 才会删它/重命名）。先改默认值。

找到：
```xml
<arg name="detector_class" default="dummy_detector.DummyDetector" />
```
改为：
```xml
<arg name="detector_class" default="qwen_vl_grounding.QwenVLGroundingDetector" />
```

同时清理该文件中所有提到 dummy 的注释（包括 `v2 vs v1 差异`、`参数说明`、`detector_class 参数支持` 等段落），保留 QwenVL 相关说明即可。

### 2.4 验证

```bash
ls ws/src/uav_vln_bringup/scripts/dummy_detector.py
# 期望输出: 文件不存在
grep -rn "dummy" ws/src/uav_vln_bringup/
# 期望输出: 无匹配（除注释里历史性引用，但 launch/scripts 里不应有功能性引用）
```

---

## 任务 4：创建新世界文件 `search_rescue.world`

**目的**：替换发黑的 `empty.world`，提供搜救任务场景。
**关键前提**：无人机由外部 PX4 SITL 流程 spawn 注入，世界文件**不写无人机模型**。

### 4.1 新建文件

路径：`simulation/px4_gazebo_classic/worlds/search_rescue.world`

**文件完整内容**（直接整个写入，下方代码就是最终成品）：

```xml
<?xml version="1.0" ?>
<!--
  search_rescue.world - 搜救任务仿真世界
  
  布局:
    - 起降平台: 黄色圆柱, 位于 (-1.01, -0.98), 无人机正下方
    - 障碍物 A: 红杆+黑色三叉支架, 位于 (0.25, 0.22), 无人机前方
    - 障碍物 B: 红杆+黑色三叉支架, 位于 (0.42, -2.15), 无人机前方偏右
  
  无人机由 PX4 SITL 外部 spawn 注入到 (-1.01, -0.98, 0.83)
  世界文件不写无人机模型
  
  光照: 高 ambient + 强太阳光 + 顶部补光, 消除 Gazebo 原生暗场
  地面: 深灰色哑光平面
  物理: ODE, 与 PX4 SITL 配置一致
-->
<sdf version="1.7">
  <world name="default">

    <!-- ========== 物理引擎 (与 PX4 SITL 兼容) ========== -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>

    <!-- ========== 场景全局光照 (关键: 消除默认黑场) ========== -->
    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.7 0.75 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 主平行太阳光 (高亮度 + 软阴影) -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.92 0.92 0.92 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>0.4 0.4 -0.9</direction>
      <cast_shadows>true</cast_shadows>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- 顶部暖白补光 (弱化物体底部死黑阴影) -->
    <light name="top_fill" type="point">
      <pose>0 0 9 0 0 0</pose>
      <diffuse>0.55 0.50 0.40 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <cast_shadows>false</cast_shadows>
      <attenuation>
        <range>30</range>
        <constant>0.5</constant>
        <linear>0.05</linear>
        <quadratic>0.005</quadratic>
      </attenuation>
    </light>

    <!-- ========== 地面: 深灰色哑光大平面 ========== -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>50</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.25 0.25 0.25 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ========================================================= -->
    <!-- 黄色起降平台 (无人机垂直正下方)                            -->
    <!-- 中心 (-1.01, -0.98, 0.01), 圆柱 r=1.3 h=0.02              -->
    <!-- 顶面白色十字标记 (两薄长方体, 厚 0.001m, z=0.021)          -->
    <!-- ========================================================= -->
    <model name="landing_pad">
      <static>true</static>
      <pose>-1.01 -0.98 0 0 0 0</pose>
      <link name="link">
        <!-- 平台主体 -->
        <collision name="pad_collision">
          <pose>0 0 0.01 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>1.3</radius>
              <length>0.02</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="pad_visual">
          <pose>0 0 0.01 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>1.3</radius>
              <length>0.02</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1.0 0.92 0.15 1</ambient>
            <diffuse>1.0 0.92 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>

        <!-- 白色十字标记 - 横条 (沿 X 轴) -->
        <visual name="cross_x">
          <pose>0 0 0.0215 0 0 0</pose>
          <geometry>
            <box>
              <size>1.8 0.12 0.001</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <!-- 白色十字标记 - 竖条 (沿 Y 轴) -->
        <visual name="cross_y">
          <pose>0 0 0.0215 0 0 0</pose>
          <geometry>
            <box>
              <size>0.12 1.8 0.001</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ========================================================= -->
    <!-- 障碍物 A: 红杆 + 三叉黑色支架                              -->
    <!-- 位置 (0.25, 0.22, 0.12)                                    -->
    <!-- 主杆: 长圆柱 RGB(0.72,0.18,0.12), 长 0.9 半径 0.05         -->
    <!-- 支架: 3 根细圆柱 RGB(0.15,0.15,0.15) 半径 0.03, 120° 分叉  -->
    <!-- ========================================================= -->
    <model name="red_tripod_A">
      <static>true</static>
      <pose>0.25 0.22 0.12 0 0 0</pose>
      <link name="link">
        <!-- 主杆 (沿 Z 轴竖直) -->
        <collision name="pole_collision">
          <pose>0 0 0.45 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.9</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="pole_visual">
          <pose>0 0 0.45 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.9</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.72 0.18 0.12 1</ambient>
            <diffuse>0.72 0.18 0.12 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>

        <!-- 支架 1 (0°方向, 向 +X 倾斜) -->
        <collision name="leg1_collision">
          <pose>0.0964 0.0000 -0.1149 0 0.6981 0.0000</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
        </collision>
        <visual name="leg1_visual">
          <pose>0.0964 0.0000 -0.1149 0 0.6981 0.0000</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>

        <!-- 支架 2 (120°方向) -->
        <collision name="leg2_collision">
          <pose>-0.0482 0.0835 -0.1149 0 0.6981 2.0944</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
        </collision>
        <visual name="leg2_visual">
          <pose>-0.0482 0.0835 -0.1149 0 0.6981 2.0944</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>

        <!-- 支架 3 (240°方向) -->
        <collision name="leg3_collision">
          <pose>-0.0482 -0.0835 -0.1149 0 0.6981 4.1888</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
        </collision>
        <visual name="leg3_visual">
          <pose>-0.0482 -0.0835 -0.1149 0 0.6981 4.1888</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ========================================================= -->
    <!-- 障碍物 B: 同款红杆 + 三叉黑色支架, 错落摆放                -->
    <!-- 位置 (0.42, -2.15, 0.12)                                   -->
    <!-- ========================================================= -->
    <model name="red_tripod_B">
      <static>true</static>
      <pose>0.42 -2.15 0.12 0 0 0</pose>
      <link name="link">
        <collision name="pole_collision">
          <pose>0 0 0.45 0 0 0</pose>
          <geometry>
            <cylinder><radius>0.05</radius><length>0.9</length></cylinder>
          </geometry>
        </collision>
        <visual name="pole_visual">
          <pose>0 0 0.45 0 0 0</pose>
          <geometry>
            <cylinder><radius>0.05</radius><length>0.9</length></cylinder>
          </geometry>
          <material>
            <ambient>0.72 0.18 0.12 1</ambient>
            <diffuse>0.72 0.18 0.12 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>

        <collision name="leg1_collision">
          <pose>0.0964 0.0000 -0.1149 0 0.6981 0.0000</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
        </collision>
        <visual name="leg1_visual">
          <pose>0.0964 0.0000 -0.1149 0 0.6981 0.0000</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>

        <collision name="leg2_collision">
          <pose>-0.0482 0.0835 -0.1149 0 0.6981 2.0944</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
        </collision>
        <visual name="leg2_visual">
          <pose>-0.0482 0.0835 -0.1149 0 0.6981 2.0944</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>

        <collision name="leg3_collision">
          <pose>-0.0482 -0.0835 -0.1149 0 0.6981 4.1888</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
        </collision>
        <visual name="leg3_visual">
          <pose>-0.0482 -0.0835 -0.1149 0 0.6981 4.1888</pose>
          <geometry>
            <cylinder><radius>0.03</radius><length>0.3</length></cylinder>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### 4.2 使用方法（启动仿真时切换世界）

启动脚本默认 `WORLD=empty`。要用新世界，启动时传入：
```bash
WORLD=search_rescue ./scripts_start_px4_sitl_gazebo.sh
```

无需修改 `scripts_start_px4_sitl_gazebo.sh`，它已经支持 `WORLD` 环境变量。

无人机的 spawn 位置由 PX4 SITL 流程决定（外部注入）。本任务**不**碰无人机配置。

### 4.3 验证

```bash
# 1. 文件创建成功
ls -la simulation/px4_gazebo_classic/worlds/search_rescue.world
# 期望: 文件存在, 大小 > 5KB

# 2. SDF 语法合法 (如果系统有 gz 工具)
gz sdf -k simulation/px4_gazebo_classic/worlds/search_rescue.world 2>&1 | head -5
# 期望: 无 Error 输出 (Warning 可忽略)

# 3. 单独用 gazebo 加载测试 (可选)
gazebo --verbose simulation/px4_gazebo_classic/worlds/search_rescue.world
# 期望: 看到黄色平台 + 两个红杆三叉障碍物 + 亮场景, 无加载错误
# 按 Ctrl+C 退出
```

### 4.4 重要约束（不要做）

- **不要**在 world 文件里加 `<model name="iris...">` 或任何无人机模型
- **不要**改 `scripts_start_px4_sitl_gazebo.sh`
- **不要**改 `empty.world`（保留作为兜底）
- **不要**修改 `<gui>` 相机视角（用户使用的是 PX4 无人机自带相机，不需要 GUI 默认视角配置）

---

## 任务 5：删除 v1 (`vlm_navigation.py` 及其 launch)

**目的**：v1 单体节点已被 `detector_node.py` + `qwen_vl_grounding.py` 完全替代，删掉减少阅读负担。

### 5.1 删除 v1 文件

```bash
rm -f ws/src/uav_vln_bringup/scripts/vlm_navigation.py
rm -f ws/src/uav_vln_bringup/launch/vlm_navigation.launch
```

**注意**：删除的是 v1 的 `vlm_navigation.launch`，不是 v2。下面会重命名 v2。

### 5.2 重命名 v2 → 主版本

```bash
mv ws/src/uav_vln_bringup/launch/vlm_navigation_v2.launch \
   ws/src/uav_vln_bringup/launch/vlm_navigation.launch
```

重命名后，编辑新的 `vlm_navigation.launch`，把内部注释里所有 "v2"、"v1 vs v2"、"替代旧版" 之类的对比描述删除，把开头注释改为简洁的当前架构说明。例如：

```xml
<!--
vlm_navigation.launch - VLM 视觉语言导航
(模块化架构: MAVROS + setpoint_bridge + detector_node)

架构:
  detector_node 支持通过 detector_class 参数切换不同检测器后端
  默认使用 qwen_vl_grounding.QwenVLGroundingDetector

使用方法:
  1. 先在终端启动 PX4 SITL + Gazebo:
      WORLD=search_rescue ./scripts_start_px4_sitl_gazebo.sh

  2. 然后在新终端启动此 launch:
      source ws/devel/setup.bash
      roslaunch uav_vln_bringup vlm_navigation.launch

  3. (可选) 通过 rqt_image_view 查看调试图像:
       rosrun rqt_image_view rqt_image_view /uav/target_debug
-->
```

文件中部 `<node ... name="detector_node">` 上面的注释，删除 "替代旧版 vlm_navigation.py" 等字样。

### 5.3 修改 `ws/src/uav_vln_bringup/CMakeLists.txt`

找到 `catkin_install_python(PROGRAMS ...)` 段落（约第 163-169 行）：
```cmake
catkin_install_python(PROGRAMS
  scripts/detector_node.py
  scripts/setpoint_bridge.py
  scripts/vlm_navigation.py
  scripts/takeoff_land.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
**删除** `scripts/vlm_navigation.py` 这一行，结果应为：
```cmake
catkin_install_python(PROGRAMS
  scripts/detector_node.py
  scripts/setpoint_bridge.py
  scripts/takeoff_land.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 5.4 修改 `README.md`

找到目录结构树（约第 27-51 行），把脚本部分从：
```
│           ├── scripts/                   # Python 脚本
│           │   ├── vlm_navigation.py      # VLM 导航主程序
│           │   ├── setpoint_bridge.py     # 目标点桥接
│           │   └── takeoff_land.py        # 起飞降落
```
改为：
```
│           ├── scripts/                   # Python 脚本
│           │   ├── detector_node.py       # 目标检测节点（模块化）
│           │   ├── base_detector.py       # Detector 抽象基类
│           │   ├── qwen_vl_grounding.py   # Qwen-VL 检测器实现
│           │   ├── setpoint_bridge.py     # 目标点桥接
│           │   └── takeoff_land.py        # 起飞降落
```

### 5.5 修改 `CLAUDE.md`

如果文件中有 v1/v2 对比描述、`vlm_navigation.py` 字样、`dummy_detector.DummyDetector` 字样，统一清理：
- 启动命令统一为 `roslaunch uav_vln_bringup vlm_navigation.launch`
- "Architecture" 段落直接描述模块化架构，不要写 v1/v2 历史
- "Detector ABC Pattern" 段落只列 `qwen_vl_grounding.QwenVLGroundingDetector` 一种实现

### 5.6 编译验证

```bash
cd ws && catkin_make 2>&1 | tail -10
# 期望: 无 Error, 编译成功
```

### 5.7 最终文件清单检查

```bash
echo "=== scripts/ ==="
ls ws/src/uav_vln_bringup/scripts/
# 期望文件: base_detector.py  detector_node.py  qwen_vl_grounding.py  setpoint_bridge.py  takeoff_land.py
# 不应有: dummy_detector.py, vlm_navigation.py

echo "=== launch/ ==="
ls ws/src/uav_vln_bringup/launch/
# 期望文件: mavros_px4_sitl.launch  README.md  takeoff_land_demo.launch  vlm_navigation.launch
# 不应有: vlm_navigation_v2.launch
```

---

## 任务 6：默认 model 改为 `qwen3.6-plus`

**目的**：项目当前用的是 `qwen3.6-plus`，不再用旧的 `qwen-vl-max`。

**说明**：只改 detector 类的**默认值**，文件名/类名/模块路径**全部保留**（`qwen_vl_grounding.py` / `QwenVLGroundingDetector` 是通用的 Qwen 视觉定位包装，具体 model 通过参数传入；改文件名会破坏这种通用性）。

### 6.1 修改 `ws/src/uav_vln_bringup/scripts/qwen_vl_grounding.py`

找到 `QwenVLGroundingDetector.__init__` 的签名（约第 48 行）：
```python
def __init__(self, api_key: str, model: str = "qwen-vl-max",
             timeout: float = 30.0) -> None:
```
改为：
```python
def __init__(self, api_key: str, model: str = "qwen3.6-plus",
             timeout: float = 30.0) -> None:
```

### 6.2 检查 launch 文件已经是 qwen3.6-plus

```bash
grep "vlm_model" ws/src/uav_vln_bringup/launch/vlm_navigation.launch
# 期望: <param name="vlm_model" value="qwen3.6-plus" />
```

如果已经是 `qwen3.6-plus`，**不要改**；如果还是 `qwen-vl-max`，把它改成 `qwen3.6-plus`。

### 6.3 不要做的修改

下面这些**不要碰**（保留为通用名）：
- ❌ 文件名 `qwen_vl_grounding.py`（不要改成 `qwen3_6_plus.py` 之类）
- ❌ 类名 `QwenVLGroundingDetector`
- ❌ `detector_class` 字符串 `"qwen_vl_grounding.QwenVLGroundingDetector"`
- ❌ 文件顶部 docstring 里的 "Qwen VL grounding detector" 描述
- ❌ `DEFAULT_PROMPT` 等常量
- ❌ `PATTERNS` 里 `Qwen2-VL format` 等注释（这些是历史 model 的兼容解析，不影响新 model）

### 6.4 验证

```bash
grep "model: str =" ws/src/uav_vln_bringup/scripts/qwen_vl_grounding.py
# 期望: model: str = "qwen3.6-plus",
```

---

## 执行顺序

按编号顺序执行：**1 → 2 → 4 → 5 → 6**

各任务相互独立，但建议按上述顺序，避免编辑冲突。

## 全部完成后的最终验证

```bash
# 1. 编译通过
cd ws && catkin_make 2>&1 | grep -E "Error|error" || echo "✓ Build OK"

# 2. 文件清单正确
ls ws/src/uav_vln_bringup/scripts/dummy_detector.py 2>&1 | grep -q "No such" && echo "✓ dummy 已删"
ls ws/src/uav_vln_bringup/scripts/vlm_navigation.py 2>&1 | grep -q "No such" && echo "✓ v1 py 已删"
ls ws/src/uav_vln_bringup/launch/vlm_navigation.launch && echo "✓ launch 存在"
ls ws/src/uav_vln_bringup/launch/vlm_navigation_v2.launch 2>&1 | grep -q "No such" && echo "✓ v2 launch 已重命名"
ls simulation/px4_gazebo_classic/worlds/search_rescue.world && echo "✓ 新世界文件存在"

# 3. env export 前缀
tail -1 docs/.env.example
# 期望: export VLM_API_KEY=sk-xxx
[[ -f .env ]] && grep "^export VLM_API_KEY" .env && echo "✓ 本地 .env 也有 export"

# 4. detector_node 默认值
grep "detector_class" ws/src/uav_vln_bringup/scripts/detector_node.py | grep "qwen_vl_grounding" && echo "✓ detector_node 默认值正确"

# 5. launch 默认值
grep "detector_class.*default" ws/src/uav_vln_bringup/launch/vlm_navigation.launch | grep "qwen_vl_grounding" && echo "✓ launch 默认值正确"

# 6. CMakeLists 无 v1 引用
grep "vlm_navigation.py" ws/src/uav_vln_bringup/CMakeLists.txt && echo "✗ CMakeLists 还有 v1 引用" || echo "✓ CMakeLists 已清理"

# 7. qwen3.6-plus 默认 model
grep 'model: str = "qwen3.6-plus"' ws/src/uav_vln_bringup/scripts/qwen_vl_grounding.py && echo "✓ qwen3.6-plus 默认值正确"
```

## 不要做的事（重要约束）

- ❌ 不要修改 `scripts_start_px4_sitl_gazebo.sh`
- ❌ 不要修改 `scripts_demo_all.sh`（基础起降 demo，不需要 VLM）
- ❌ 不要往任何脚本/launch 里加 `source .env`（本阶段用户手动 source）
- ❌ 不要在 world 文件里加无人机模型
- ❌ 不要保留 v1 文件（不是"标记废弃"，是物理删除）
- ❌ 不要改 `empty.world`
- ❌ 不要改 `setpoint_bridge.py`、`takeoff_land.py`、`base_detector.py`
- ❌ 不要改 `qwen_vl_grounding.py` 的文件名/类名/docstring（只改默认 model 参数值）
- ❌ 不要触碰 `mavros_px4_sitl.launch`、`takeoff_land_demo.launch`
- ❌ 不要在 world 文件里加 `<gui>` 相机配置（用户用的是无人机自带相机）
