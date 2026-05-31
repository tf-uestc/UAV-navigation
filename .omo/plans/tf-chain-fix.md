# Plan: TF 链修复

## Metadata
- **Created**: 2026-05-31
- **Status**: ready
- **Related**: F3 (TF 链改造), F2 (VLM Grounding 前置依赖)

## Goal
修复 MAVROS TF 广播 + 相机话题名，使 TF 树连通：`map → odom → base_link → camera_link → camera_link_optical`

## Background
TF 树有三棵断开的子树。根因：MAVROS 默认 `local_position/tf/send=false`。launch 已加 `<rosparam>` 设为 `true`，但被 `<include mavros/px4.launch>` 内的 `px4_config.yaml` 覆盖。`global_position/tf/send` 也未开启。相机话题名未更新。

## Scope
- **IN**: mavros_px4_sitl.launch、vlm_navigation.launch
- **OUT**: Python 代码、PX4 配置、Gazebo world、EGO-Planner

## TODOs

- [x] 1. 修复 mavros_px4_sitl.launch — 将 rosparam 移到 include 后面 + 新增 global_position TF
- [x] 2. 修复 vlm_navigation.launch — 更新相机话题名
- [x] 3. 验证 TF 连通性 — tf_echo map camera_link_optical 有输出
- [x] 4. 验证相机话题 — rostopic echo 可读取

## Details

### 1. 修复 mavros_px4_sitl.launch

**文件**: `ws/src/uav_vln_bringup/launch/mavros_px4_sitl.launch`

**改动 A**: 将第 8-10 行 rosparam（local_position TF）从 include 前面移到第 15 行 `</include>` 后面，注释说明"在 px4.launch 之后加载，覆盖 YAML 默认 false"

**改动 B**: 在 `</include>` 之后新增 global_position TF rosparam：
```xml
<rosparam param="/mavros/global_position/tf/send" subst_value="True">true</rosparam>
<rosparam param="/mavros/global_position/tf/frame_id" subst_value="False">map</rosparam>
<rosparam param="/mavros/global_position/tf/child_frame_id" subst_value="False">odom</rosparam>
```

### 2. 修复 vlm_navigation.launch

**文件**: `ws/src/uav_vln_bringup/launch/vlm_navigation.launch`

**改动**: 第 48-49 行相机话题名：
```
/camera/rgb/image_raw        →  /iris_depth_camera/camera/rgb/image_raw
/camera/depth/image_raw      →  /iris_depth_camera/camera/depth/image_raw
```

### 3. 验证 TF 连通性

```bash
source /opt/ros/noetic/setup.bash && source ~/毕业设计/ws/devel/setup.bash
roslaunch uav_vln_bringup mavros_px4_sitl.launch &
sleep 15
rosrun tf tf_echo map base_link -n 3
rosrun tf tf_echo map camera_link_optical -n 3
```

### 4. 验证相机话题

```bash
rostopic info /iris_depth_camera/camera/depth/image_raw
rostopic echo /iris_depth_camera/camera/depth/image_raw/header -n 1 | grep frame_id
```

## Final Verification Wave

- [x] F1. Oracle review: verify all changes are correct and complete
- [x] F2. Code review: check launch XML syntax, rosparam namespace paths
- [x] F3. Security review: no API keys leaked, no unsafe config changes
- [x] F4. Hands-on verification: run launch, verify tf_echo map camera_link_optical succeeds

## Acceptance Criteria
1. `rosrun tf tf_echo map camera_link_optical` 输出连续数值（不报 "not part of the same tree"）
2. `rosparam get /mavros/local_position/tf/send` 返回 `true`
3. `/tf` topic 上有持续消息发布
4. 相机话题路径匹配实际 Gazebo 发布

## Risks
- **Time jump 错误**: MAVROS SITL 启动瞬态，通常自愈
- **global_position 插件**: 若被 mavros blacklist，需额外开启
