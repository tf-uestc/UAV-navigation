# 第5至6项 项目场景与RGB-D相机静态验证

日期：2026年11月2日。用户确认使用现有search_rescue.world，而非另建圆环场景。任务5和6通过本次规定的静态验收；全程未解锁、未起飞、未发布运动指令，未调用云端检测API。本批不需要人工介入。

## 实际运行的组合

- 项目世界：simulation/px4_gazebo_classic/worlds/search_rescue.world，包含黄色停机坪、red_tripod_A和blue_tripod_B。
- 飞机：PX4 v1.14.0子模块提供的iris_depth_camera，包含标准iris与前置depth_camera；项目目录覆盖depth_camera的ROS插件配置。
- 相机机体安装位置：base_link前方约0.1 m，无额外0.035 m叠加。Gazebo运行时链接真值已核对；include pose覆盖了被包含模型的pose。
- GPU：VirtualGL egl0；ROS图形插件由显式加载libgazebo_ros_api_plugin.so的gzserver初始化。
- ROS时钟：/use_sim_time=true，由/gazebo发布/clock。MAVROS本地UDP接收14540，PX4对端14580。

旧scripts_start_px4_sitl_gazebo.sh、setpoint_bridge.py和empty.world均未改动。新启动入口为tools/deployment/launch_project_rgbd.sh。PX4和EGO算法源码没有在本批中修改。

## 根据实测修正的配置

1. search_rescue.world把gravity和magnetic_field错误地放在physics内部，Gazebo报告bad any_cast，实际重力为默认-9.8。将二者移至world层级，保留原定数值，重启后实测重力为-9.8066，相关bad any_cast错误消失。顺便修正world注释中的出生位置负号，实际出生配置仍为(1.01,0.98,0.83)。依据为服务器/usr/share/sdformat9/1.7/world.sdf与physics.sdf模式文件。
2. 相机输出的图像、CameraInfo和点云是RDF光学数据，但frameName原为camera_link。改为camera_link_optical；相机物理安装坐标仍为camera_link。
3. OpenNI插件点云按(width-1)/2、(height-1)/2反投影，而默认CameraInfo主点是(width+1)/2、(height+1)/2。显式设置Cx=319.5、Cy=239.5，使640×480的CameraInfo与点云一致。修改前抽样最大反投影误差约0.198 m，修改后约1.7e-6 m。依据：[OpenNI点云实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp)及[CameraInfo默认实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_camera_utils.cpp)。
4. 把相机SDF的XML声明移到首行，并使用插件支持的distortionK1等字段名；零畸变设定保持不变。光学TF使用精确四元数(-0.5,0.5,-0.5,0.5)，避免截断的欧拉角。MAVROS默认对端端口更新为此PX4基线实际使用的14580。
5. PX4存在iris_depth_camera模型文件，但没有相应的顶层make机型目标。新增启动脚本直接加载该SDF，并显式选择已验证的Iris airframe 10015，不改PX4目标列表，不在world中写入无人机。

## 验证方法和实测结果

tools/deployment/validate_project_rgbd.py只读订阅图像、CameraInfo、点云和MAVROS状态，精确同步后使用Gazebo链接真值、已知红色圆柱半径0.05 m及位置(6,1.5)验证投影；失败返回非零退出码。该脚本针对本场景的固定目标，不是通用检测器。

| 检查 | 修正后首次验证 | 从仓库脚本重新启动后 |
| --- | --- | --- |
| 验收断言 | 15项全部通过 | 15项全部通过 |
| RGB / 深度 / 点云 | 各约10 Hz | 各约10 Hz |
| CameraInfo | 约8.99 Hz | 约9.29 Hz |
| 图像与参数及点云五路同时间戳 | 89组 | 93组 |
| RGB / 深度编码 | rgb8 / 32FC1，单位米 | 相同 |
| 分辨率 | 640×480 | 相同 |
| fx、fy / cx、cy | 343.159073 / 319.5、239.5 | 相同 |
| 输出帧 | 全部camera_link_optical | 相同 |
| 稳态时间戳 | 单调推进，约9.9秒 | 相同 |
| MAVROS | connected=true，armed=false | 相同 |
| 点云与深度反投影最大误差 | 约1.61e-6 m | 约1.70e-6 m |
| 红色杆表面几何误差P95 | 约0.26 mm | 约0.25 mm |
| TF重建与Gazebo真值投影最大差 | 约0.32 mm | 约0.32 mm |

上述毫米级数值仅表示同一仿真静态几何的数值一致性，不代表视觉模型检测或真实世界定位精度。红色目标深度约4.843 m；重建点为圆柱朝向相机的表面，而非目标中心，不能用表面x≈5.954与中心x=6.0的差值直接判为定位失败。

TF链map→base_link→camera_link→camera_link_optical在图像采集时间可查询。项目tf_healthcheck.py实际运行，确认4组必需变换就绪；随后以timeout结束这个长期驻留的检查进程，超时退出124是预期清理方式。此结果没有替代速度坐标和运动场景验证。

本批修改后离线回归：81项测试、0失败、0错误。git diff --check通过；受保护的旧启动脚本、飞控桥和empty.world无差异。

## 如何启动与验证

服务器使用独立新终端，先确认没有旧仿真实例；脚本会拒绝重复启动。前台运行可直接查看输出日志目录，Ctrl+C结束它启动的子进程：

```bash
cd /root/autodl-tmp/uav/UAV-navigation
bash tools/deployment/launch_project_rgbd.sh
```

默认日志放在/root/autodl-tmp/uav/logs/project-rgbd-时间戳。路径可通过UAV_ROOT、UAV_PROJECT_DIR、PX4_AUTOPILOT_DIR、UAV_RUN_DIR覆盖；本机默认配置保持ROS→EGO→业务overlay加载顺序，不激活PX4构建虚拟环境。

另开终端执行只读验证：

```bash
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/noetic/setup.bash
cd /root/autodl-tmp/uav/UAV-navigation
/usr/bin/python3 tools/deployment/validate_project_rgbd.py \
  --phase repeat --output-dir /root/autodl-tmp/uav/records/rgbd-repeat
```

目前后台保留了重新启动后的项目场景，便于用户通过VNC查看；未解锁。启动脚本会保存supervisor.pid，停止时应先核实PID与进程命令再发信号，避免误停其他实例。

## 已知限制和下一步

- 地面停机状态下，相机离平台很近，0.2 m近距离截断使本帧有效深度比例约19.4%；红蓝目标在有效范围。未改变相机安装位置以提高该比例，飞行高度下的有效深度需后续测试。
- 修改前第一次订阅曾收到旧的缓存RGB帧；稳态验收先预热2秒并检查时间戳。业务节点仍需在U09按采集时间拒绝陈旧数据，本次没有宣称解决所有缓存与超时问题。
- CameraInfo实测速率约9 Hz，并非严格10 Hz；十秒内取得89/93组精确同步数据。需要满速持续同步的下游任务应再次评估，不以话题配置中的10替代实测。
- 世界重力/磁场设置错误已修正；日志仍有上游PX4启动Illegal number、/dev/led0、SDF兼容及Gazebo ROS的rendering始终启用提示。保持原始日志，不将其一概认定为已解决。
- 仅验证原始传感器与TF静态几何。未启动EGO建图/规划、检测API或全导航launch，未验证运动时定位、遮挡、时序异常和180度旋转。后续第7项及以后仍待审核。

## 文件与版本状态

源码基线仍为业务d9a74ac、PX4 b8c541d、EGO bfda512。本批配置和两个工具文件尚未提交；精确内容由records/2026-11-02/step56-2026-11-02/source-sha256.json及config-changes.patch对应。原始数据、前后对比、独立重启、TF和离线回归日志均在该目录。本地与服务器五个源文件SHA256一致。未提交、未推送GitHub；未处理用户原有的其他文件删除或改动。
