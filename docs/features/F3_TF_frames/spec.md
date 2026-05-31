# F3 — Spec: TF 链与坐标系修正

## 1. 目标

把 `vlm_navigation.py:DepthProjector.camera_to_world_approx` 里"只用 yaw、忽略相机偏移、混淆光学系"的近似实现，替换为**完整的 ROS TF 链**：相机光学系 → 机体系 → 世界系全部由 `tf2` 自动处理。

同时让 TF 树设计同时兼容 **PX4 SITL（仿真真值）** 和 **室内动捕（OptiTrack/Vicon）** 两种定位源，切换时下游模块零改动。

## 2. 范围

In scope:
- 完整 TF 树：`map → odom → base_link → camera_link → camera_link_optical`
- 在 launch 里补齐 `base_link → camera_link` 的 static transform（如果 SDF 没发布）
- 仿真 + 动捕的 frame 命名一致；动捕路径预留 `mavros/vision_pose/pose` 接入
- 提供一个 `tf2_buffer.transform()` 的标准用法封装供 F2 调用
- TF 树健康检查工具：launch 起来能 echo 出关键变换

Out of scope:
- 多机坐标系联合（future work）
- VIO / LIVO 接入（plan.md 选了动捕，VIO 留 future work）
- 动捕厂商 SDK 安装（属于环境搭建，不在代码范围）
- 真正搭好动捕系统（毕设主线是仿真，动捕在 F3 里只是预留接口）

## 3. 输入 / 输出契约

**输入**：
- `/mavros/local_position/pose` (`PoseStamped`) — 来自 PX4，由 mavros 同步发到 TF 作为 `map → base_link`
- 相机的 `base_link → camera_link → camera_link_optical` 静态变换 — 由 SDF 或 launch 内的 `static_transform_publisher` 发布
- （动捕路径才会用到）`/vrpn_client_node/<rb>/pose` 或 `/optitrack/.../pose` — remap 到 `/mavros/vision_pose/pose`

**输出**：
- 完整 TF 树（不是话题，而是 `/tf` 上能查到 `map ↔ camera_link_optical` 的变换）
- `tf2_buffer.transform(point, "map")` 这个调用在任何节点里都能成功

## 4. 帧约定（必须严格遵守）

参考 [REP-103](https://www.ros.org/reps/rep-0103.html)（坐标轴）+ [REP-105](https://www.ros.org/reps/rep-0105.html)（frame 命名）：

| frame_id | 轴定义 | 谁来发布 | 是否随时间变化 |
|---|---|---|---|
| `map` | ENU（东-北-天） | mavros / 动捕（间接） | 否（全局） |
| `odom` | ENU | mavros | 短期连续，长期可能漂 |
| `base_link` | FLU（前-左-上），机体 | mavros | 是 |
| `camera_link` | FLU，相机机械原点 | SDF / static publisher | 否（静态偏移） |
| `camera_link_optical` | x→右 y→下 z→前，相机光学系 | SDF / static publisher | 否（相对 camera_link 固定旋转） |

**`camera_link_optical` 是 ROS 视觉处理的标准 frame**，所有图像 / 深度的 `header.frame_id` 必须填这个。F2 里像素回投得到的点也直接打在这个 frame 里。

⚠️ 当前 `iris_depth_camera.sdf`（如果存在）的相机插件 `<frameName>` 字段决定 frame 名。需要核对：
- 如果填的是 `camera_link`：F3 必须再加 `camera_link → camera_link_optical` 的静态旋转（+90° y, +90° z 或等价四元数）
- 如果直接填了 `camera_link_optical`：那 `camera_link` 这一层可以省，但 base_link 到 optical 的偏移要算对

## 5. 验收指标

| 指标 | 验收 |
|---|---|
| `rqt_tf_tree` 能看到从 `map` 到 `camera_link_optical` 的连续路径 | ✅ |
| `rosrun tf tf_echo map camera_link_optical` 数值随飞机移动正确变化 | ✅ |
| 静止悬停时 F2 输出的目标点 z 误差（vs 真值） | ≤ 0.1 m |
| 飞行中（含俯仰 ±15°）F2 输出的目标点误差 | ≤ 0.5 m（旧版 yaw-only 在俯仰大时误差可达 1+ m） |
| 仿真 launch 与"动捕模拟"launch 跑同一份测试代码，结果一致 | ✅ |

> 动捕模拟：录一份仿真真值 rosbag，重放成 `/mavros/vision_pose/pose` 喂给一个空的 PX4，验证下游 frame 一致。

## 6. 不变量

- F2 / F4 / setpoint_bridge **绝对不能**自己写四元数旋转代码，全部走 `tf2_buffer.transform()`
- 所有图像消息的 `header.frame_id` 必须填 `camera_link_optical`，不能空也不能错
- `map` 在仿真和动捕里**含义一致**：起飞点为原点的 ENU 坐标系（或动捕系统里的固定 origin）

## 7. 风险

| 风险 | 影响 | 兜底 |
|---|---|---|
| Gazebo 相机插件没发布 TF | TF 链断 | launch 里补 `static_transform_publisher` |
| `/depth/camera_info` 的 frame_id 与 SDF 不一致 | 相机内参对不上 frame | F3 检查脚本里加断言，启动失败给出清晰错误 |
| 动捕重定位（机器突然位置跳变） | 飞控融合后位置突变 | 动捕路径加位置变化阈值滤波（属于动捕集成的事，不在 F3 主体） |
| TF lookup 超时 | F2 当帧失败 | `tf_buffer.transform(..., rospy.Duration(0.1))` 捕异常返回 None |
| MAVROS 是否发 `map` 还是 `world` | 帧名不一致全链路炸 | mavros launch 里固定用 `map`（mavros 默认参数 `world_frame_id=map`） |

## 8. 与现状的差异

`vlm_navigation.py:DepthProjector.camera_to_world_approx`（67-93 行）整段函数：**删除**。所有调用它的地方改成走 `tf2_buffer.transform()`。

`mavros_px4_sitl.launch` 需要核对：mavros 默认会在 `/mavros/local_position/pose` 之外发布 `map → base_link` TF；如果当前没发，需要打开对应配置。
