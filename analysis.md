项目全面审查报告
  
  10 个审查维度已全部完成（其中 7 个由并行 agent 完成，3 个手动补充）。按严重程度分类汇总如下：

  ---
  一、BUG（需修复）
  
  1. detector_node.py 线程锁使用反了

  文件: scripts/detector_node.py:137-144 vs 176-179

  回调 _cb_rgb/_cb_depth/_cb_cam_info 写数据时不加锁，但 _run_detection 读数据时加锁——锁的方向反了。写入方必须也持锁，否则读写可能撕裂。此外每次收到 instruction 都 threading.Thread 创建新线程，多个检测线程可能并发运行。

  2. ego_adapter.py B-spline 控制点越界

  文件: scripts/planning/ego_adapter.py:135-143

  total_pts = max(2, len(pts)) 当 pts 为空时 total_pts=2，但循环 pts[i] 会在 i=0 时 IndexError。虽然上面有 if not pts: return 的守卫，但空列表和长度 1 的列表仍有风险。

  3. ego_adapter.py 在回调线程中阻塞

  文件: scripts/planning/ego_adapter.py:81-95

  plan() 方法用 rospy.sleep(0.05) 轮询最多 10 秒。它被 planner_node._on_target() 调用——这是 subscriber 回调，在 ROS spinner 线程中执行。阻塞会饿死同节点的所有其他回调（pose 更新、命令等）。

  4. trajectory_follower.py 非原子更新

  文件: scripts/trajectory_follower.py:52-54

  _on_traj 回调先写 self._traj 再写 self._traj_start_time，两步之间 _tick 定时器可能读到新轨迹但旧时间戳，导致 elapsed 值错误，无人机瞬间跳到轨迹末尾。

  5. trajectory_follower.py 无轨迹时不发 setpoint

  文件: scripts/trajectory_follower.py:65-69

  当 self._traj is None 时只打日志，不发布任何 PoseStamped。但 PX4 OFFBOARD 模式要求持续 setpoint 流（>2Hz），停发超过 500ms 会触发 failsafe（降落/返航）。

  6. qwen_vl_grounding.py bbox 归一化启发式有误

  文件: scripts/qwen_vl_grounding.py:192

  条件 max(coords) <= 1000 and (img_w <= 1000 or img_h <= 1000) 对于常见的 1920x1080 图像（两个维度都 >1000）不会触发归一化——但 Qwen-VL 始终输出 [0,1000] 坐标。结果：bbox 坐标将被当作像素坐标直接使用，检测框严重偏左上角。

  7. 仿真模型 model.config 名称错误

  文件: simulation/px4_gazebo_classic/models/depth_camera/model.config

  模型名写成了 "Hokuyo"（一款激光雷达），实际应该是 "depth_camera"。Gazebo 的模型注册表会使用此名称。

  8. 仿真模型 mesh 引用缺失

  文件: simulation/px4_gazebo_classic/models/depth_camera/depth_camera.sdf

  引用 model://realsense_camera/meshes/realsense.dae，但 realsense_camera 模型不存在于项目中，Gazebo 会报找不到模型错误。

  ---
  二、ISSUE（需关注）

  9. detector_node.py 未校验 cam_info

  文件: scripts/detector_node.py:179,201

  cam_info 可能为 None（还没收到 CameraInfo 消息），直接传给 detector.detect() 会导致 K[0] 访问 NoneType 崩溃。

  10. setpoint_bridge.py TAKEOFF 阶段阻塞

  文件: scripts/setpoint_bridge.py:256-295

  起飞流程中有多段 for/while + rate.sleep() 阻塞循环（预发布 2 秒、等待高度最长 15 秒、降落 2 秒），期间所有回调被阻塞，goal/state_cmd 消息积压。

  11. planner_node.py plan() 无 try-except

  文件: scripts/planner_node.py:111

  self.planner.plan(start, goal) 若抛异常，整个回调崩溃，ROS spinner 线程可能静默死亡。

  12. B-spline 转换是线性插值而非真正的 B-spline 求值

  文件: scripts/planning/ego_adapter.py:123-150

  丢弃了 order、knots、yaw_pts 信息，直接对控制点做线性插值。这在数学上不正确——生成的轨迹不连续不光滑，在控制点稀疏时偏差显著。

  13. EGO 适配器不支持重规划

  文件: scripts/planning/ego_adapter.py

  只做一次目标发布 + 等待轨迹。EGO-Planner 的 FSM 支持运动中重规划（障碍物更新时），但当前适配器不会在飞行中重新触发，避障能力未被真正使用。

  14. qwen_vl_grounding.py confidence 度量不准确

  文件: scripts/qwen_vl_grounding.py:96

  confidence 实际测量的是深度有效覆盖率，不是检测置信度。名称有误导性。

  15. SDF 版本不一致

  search_rescue.world 用 SDF 1.7，empty.world 和 depth_camera.sdf 用 SDF 1.5。Gazebo Classic 对 1.7 支持可能不完整。

  16. 深度相机插件过时

  文件: depth_camera.sdf

  使用 libgazebo_ros_openni_kinect.so（2012 年的 Kinect 驱动），在标准 ROS Noetic 安装中可能不可用。

  17. 多个 Python 脚本缺少可执行权限

  planner_node.py、tf_healthcheck.py、trajectory_follower.py、qwen_vl_grounding.py、base_detector.py 都缺少 +x 权限。虽然 catkin_install_python 安装时会加，但 rosrun 直接运行时会失败。

  18. MAVROS TF 配置冗余

  文件: launch/mavros_px4_sitl.launch:12-19

  同时发布 map→base_link 和 map→odom，可能导致 TF 树过约束。REP-105 标准是 map→odom + odom→base_link。

  ---
  三、文档不一致

  19. README.md 目录结构过时

  - 缺少 planner_node.py、trajectory_follower.py、tf_healthcheck.py、planning/ 目录
  - 缺少 ego_planner.launch、vlm_navigation_full.launch、test_planner_*.launch
  - models 目录写的是 iris_depth_camera/ 但实际只有 depth_camera/
  - 缺少 search_rescue.world

  20. launch/README.md 严重过时

  只列了 2 个 launch 文件，实际有 7 个。

  21. package.xml 缺少依赖

  缺少 depth_image_proc（ego_planner.launch 使用的 nodelet）和 tf（setpoint_bridge.py 使用 tf.transformations）。

  22. CLAUDE.md 中 scripts/ 目录结构列了 detection/ 和 planning/ 两层

  实际文件结构是 scripts/base_detector.py + scripts/qwen_vl_grounding.py（扁平）+ scripts/planning/（只有 planning 是子目录），与文档不符。

  ---
  四、改进建议

  ┌─────┬────────────────────────────────────────────────────────────────────┬────────────────────────┐
  │  #  │                                建议                                │          影响          │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ A   │ detector_node 加入线程池（max_workers=1）避免并发检测              │ 防止多线程竞争         │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ B   │ ego_adapter 把 plan() 移到独立线程，用 threading.Event 通知        │ 解除回调阻塞           │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ C   │ trajectory_follower 无轨迹时发布当前 hold position setpoint        │ 防止 OFFBOARD failsafe │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ D   │ ego_adapter 实现 B-spline 真正求值（用 scipy.interpolate.BSpline） │ 轨迹质量               │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ E   │ ego_adapter 支持持续重规划（订阅新障碍物时重发 goal）              │ 真正避障               │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ F   │ tf_healthcheck 增加 base_link→camera_link 检查                     │ 更完整的启动检查       │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ G   │ 统一使用 rospy.Duration 编码轨迹时间而非滥用 header.stamp          │ 减少混淆               │
  ├─────┼────────────────────────────────────────────────────────────────────┼────────────────────────┤
  │ H   │ ego_planner_params.yaml 相机内参与实际 SDF 模型校验对齐            │ 深度映射正确性         │
  └─────┴────────────────────────────────────────────────────────────────────┴────────────────────────┘