# F3 — Tasks

## 0. 诊断（先做这一步！）

- [ ] 启动现状的 `vlm_navigation.launch`，跑 `rosrun rqt_tf_tree rqt_tf_tree` 截图
- [ ] 跑 `rosrun tf tf_echo map base_link` 和 `rosrun tf tf_echo map camera_link` 看哪些已经存在
- [ ] 跑 `rostopic echo /camera/depth/image_raw/header -n 1` 看 frame_id 实际是什么
- [ ] 把上面截图存到 `docs/features/F3-tf-frames/screenshots/before/`

诊断结果决定下面要做哪些；不要凭直觉补 TF。

## 1. mavros TF 发布

- [ ] 拷一份 mavros 默认 `px4_config.yaml` 到 `ws/src/uav_vln_bringup/config/px4_config.yaml`
- [ ] 修改 `local_position.tf.send: true`、`frame_id: map`、`child_frame_id: base_link`
- [ ] 在 `mavros_px4_sitl.launch` 里加 `<rosparam command="load" file="$(find uav_vln_bringup)/config/px4_config.yaml"/>`
- [ ] 重启验证 `rosrun tf tf_echo map base_link` 有数据

## 2. 相机静态 TF

- [ ] 找到实际使用的 SDF 文件（`$PX4_AUTOPILOT_DIR/Tools/sitl_gazebo/models/iris_depth_camera/`）
- [ ] 看相机插件 `<frameName>` 是什么
- [ ] 如果不是 `camera_link_optical`：在 `mavros_px4_sitl.launch` 里加两段 `static_transform_publisher`（plan.md §4）
- [ ] 验证 `rosrun tf tf_echo map camera_link_optical` 成功
- [ ] 飞行时手动测：让飞机俯仰 +15°，看变换是否包含俯仰分量（之前 yaw-only 的版本不会包含）

## 3. tf_healthcheck 脚本

- [ ] 写 `scripts/tf_healthcheck.py`（plan.md §7 骨架）
- [ ] 加到 `CMakeLists.txt:catkin_install_python` 列表
- [ ] 在新 launch 里以 `required="true"` 启动，让 TF 没准备好就整体退出

## 4. 替换 yaw-only 代码

- [ ] 在 `vlm_navigation.py` 删掉 `DepthProjector.camera_to_world_approx`
- [ ] 加 `tf2_ros.Buffer + TransformListener` 到 `__init__`
- [ ] 写 `camera_point_to_map` 方法（plan.md §5 骨架）
- [ ] 把 `_run_vlm_cycle` 里调用点改成新方法
- [ ] 失败路径：返回 None → 当前周期跳过，不发 goal

> 这一步会让旧版 launch 也用上新 TF，等于顺便把老链路也修了，不破坏向后兼容。

## 5. 动捕预案 launch

- [ ] 写 `mavros_mocap.launch`（plan.md §6 模板）—— **不需要测试**，只为留接口
- [ ] 在 `docs/features/F3-tf-frames/mocap_setup.md` 记下：真接入时 PX4 参数（`EKF2_AID_MASK`、`EKF2_HGT_MODE`）、网络配置、VRPN 服务器 IP

## 6. 验证

- [ ] 仿真悬停场景：放一个固定红色立方体（位置在 Gazebo 里查得到真值），让 F2 输出 3D 点
  - 旧版（yaw-only）：记录误差
  - 新版（tf2）：记录误差
  - 写到 `docs/experiments/F3_tf_accuracy.md`
- [ ] 飞行场景：让飞机来回飞 3 次，记录俯仰下的目标点抖动
- [ ] 录一份完整 rosbag（`/tf`、`/tf_static`、`/camera/*`、`/uav/target_world`）做 demo

## 7. 文档

- [ ] 在 `docs/features/F3-tf-frames/screenshots/after/` 存最终 TF 树截图
- [ ] 把 spec.md §5 表格的实测值填进去

## 完成标准

- spec.md §5 五项全部达标
- F2/F4 代码里**找不到任何手写四元数旋转**（grep 一下 `quaternion_multiply`/`atan2.*z.*y`）
- 老 launch + 新 launch 都能起来，TF 树健康
