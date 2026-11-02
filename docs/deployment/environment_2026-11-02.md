# 2026年11月2日 AutoDL 环境与基础仿真验证记录

本记录对应用户批准的任务1至4：记录环境、编译Gazebo插件、标准Iris仿真、MAVROS基础通信。四项通过；不代表项目完整导航链路通过。记录时飞机未解锁，未执行起飞、OFFBOARD或目标检测API调用。

## 版本与资源

| 组件 | 固定版本或实际配置 |
| --- | --- |
| 系统 | Ubuntu 20.04.4 LTS，x86_64 |
| 实例资源 | cgroup限制10核、30 GiB内存；不是free显示的宿主机251 GiB |
| GPU | NVIDIA GeForce RTX 3080 Ti，12288 MiB；驱动595.71.05 |
| 业务仓库 | fix/navigation-stability；d9a74ac8aac6dadb95a7d1d5a9b1024cc7aeed8a |
| EGO | uav-integration；bfda51284c8c1b476043255a8145ef925a3778a5 |
| PX4 | uav-sitl；v1.14.0；b8c541dd7277ed735139d7d1bfb829d61fbe29fb |
| Classic子模块 | Tools/simulation/gazebo-classic/sitl_gazebo-classic；2e3ed9bfb04d8865e59380afa575dd37b0d6c8e0 |
| ROS / Gazebo / MAVROS | Noetic / Classic 11.15.1 / 1.20.1 |
| 图形 | TurboVNC 3.3.1，VirtualGL 3.1.5，DISPLAY=:1，vglrun -d egl0 |

PX4 v1.14.0是此次新部署的基线，不能认定与旧电脑环境完全一致。EGO的src/CMakeLists.txt符号链接由原来的Kinetic指向改为/opt/ros/noetic/share/catkin/cmake/toplevel.cmake，是已有catkin工作空间适配差异，已保存patch；EGO算法源码未在本次步骤中修改。

## 各环节应使用的环境

| 使用场景 | Python与依赖 | 使用边界 |
| --- | --- | --- |
| ROS节点、EGO消息、MAVROS、检测节点 | /usr/bin/python3，3.8.10；APT NumPy 1.17.4、OpenCV 4.2.0、requests 2.22.0 | 不激活Conda或PX4虚拟环境。ROS→EGO→业务依次source。检测模块导入已通过，真实API调用、RGB-D/TF与模型效果未验证。 |
| PX4与Classic插件构建 | /root/autodl-tmp/uav/venvs/px4-py38/bin/python；EmPy 3.3.4、NumPy 1.23.5；pip check通过 | venv启用system-site-packages；保持系统APT包原样。requirements保留PX4版本，仅修正matplotlib>=3.0.*的无效版本表达式；约束nunavut<2。 |
| 离线测试 | /root/autodl-tmp/uav/venvs/offline-py38/bin/python | 之前81项测试通过；与真实ROS集成、飞行验收分开记录。 |
| Conda基础环境 | /root/miniconda3 | 保留给Jupyter等已有用途，本次ROS与PX4不依赖它。 |

当前检测实现通过requests调用云端DashScope接口，没有在服务器部署本地模型。本次不额外安装PyTorch、CUDA Toolkit或cuDNN。nvidia-smi显示的CUDA版本表示驱动支持能力，不能据此认定已安装对应Toolkit。后续若改为本地模型推理，应单独建立环境并重新验证ROS图像数据接口。

ROS终端建议使用：

```bash
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH
source /opt/ros/noetic/setup.bash
source /root/autodl-tmp/uav/ego-planner/devel/setup.bash
source /root/autodl-tmp/uav/UAV-navigation/ws/devel/setup.bash
command -v python3
```

PX4构建在独立子shell执行，结束后恢复调用终端环境：

```bash
(
  export PATH=/usr/sbin:/usr/bin:/sbin:/bin
  unset PYTHONHOME PYTHONPATH
  source /root/autodl-tmp/uav/venvs/px4-py38/bin/activate
  cd /root/autodl-tmp/uav/PX4-Autopilot
  make -j4 j=4 PYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python" px4_sitl_default
)
```

下载GitHub依赖失败时，只在需要的子shell中source /etc/network_turbo；APT或非GitHub资源避免继承该代理。不要关闭TLS证书校验。

## Gazebo插件构建

PX4的Classic外部工程按宿主机资源计算并发，不能仅依赖顶层make -j4。本次使用相同配置在标准输出目录独立构建，未修改PX4源码：

```bash
cmake -S Tools/simulation/gazebo-classic/sitl_gazebo-classic \
  -B build/px4_sitl_default/build_gazebo-classic -G Ninja \
  -DCMAKE_INSTALL_PREFIX=/usr/local -DSEND_ODOMETRY_DATA=ON \
  -DGENERATE_ROS_MODELS=ON -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
cmake --build build/px4_sitl_default/build_gazebo-classic -- -j4
```

恢复编译退出码0，日志含GAZEBO_PLUGIN_BUILD_OK。对libgazebo_mavlink_interface.so、libgazebo_motor_model.so、libgazebo_imu_plugin.so、libgazebo_user_camera_plugin.so执行ldd，均无not found。以后直接运行make px4_sitl gazebo-classic仍可能触发外部工程的自动并发；优先按此记录独立构建，再用启动脚本运行。

## 标准Iris与MAVROS验证

仿真为PX4上游标准iris和empty.world，尚未接入业务iris_depth_camera、自定义场景或导航launch。启动脚本：服务器/root/autodl-tmp/uav/records/start_standard_iris_2026-11-02.sh；在运行前检查已有进程，避免重复启动。

Gazebo服务器与客户端通过VirtualGL使用egl0。vglrun glxinfo确认渲染器为RTX 3080 Ti。桌面截图显示飞机、持续推进的仿真时间，截图时实时因子约1.00。PX4日志包含Simulator connected on TCP port 4560、Startup script returned successfully和Ready for takeoff。

MAVROS配置：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:='udp://:14540@127.0.0.1:14580'
```

12秒采样结果：

| 检查 | 实测结果 |
| --- | --- |
| 状态 | 13条，全部connected=true；armed=false；mode=AUTO.LOITER |
| 本地位姿 | 360条，约29.99 Hz；frame=map；时间戳推进11.97秒 |
| IMU | 600条，约49.98 Hz；frame=base_link；时间戳推进11.98秒 |
| 四元数 | 位姿和IMU模长约1，采样最小约0.99999996、最大约1.00000010 |
| 加速度 | 最后一帧z约9.81 m/s²，与静置时含重力的读数相符 |

位姿z约-0.20 m来自此基线估计原点，不能直接当作世界真值或相机外参标定结论。以上是短时基础连通验证，不替代长时稳定性、速度坐标核对或飞行验收。

## 已知提示及后续边界

启动日志存在非致命提示：rcS的Illegal number、/dev/led0不可用、SDF属性兼容、Gazebo GUI首次创建配置和XDG_RUNTIME_DIR默认值、MAVROS命令520提示。它们未阻止本次连接与数据验证，原始日志保留；未宣称已经定位或修复全部提示。未改动飞控参数以掩盖提示。

本节前述标准Iris数据对应先前任务1至4。随后用户批准任务5至6，并确认使用search_rescue.world；项目iris_depth_camera及RGB-D静态TF验证现已通过，详细修正和证据见rgbd_validation_2026-11-02.md。U00环境与完整模型基线已通过；U01安装空间等、U03动态坐标场景及EGO连续重规划/取消、唯一控制出口、真实检测API、起飞和180度旋转仍未完成。

## 记录位置与备份

仓库docs/deployment/environment_2026-11-02.md是可版本管理的环境说明；同目录records/2026-11-02保存验证JSON、构建/运行日志、环境锁定清单和截图。docs/阶段性总结/阶段总结.docx保留阶段结论，不替代详细复现记录。

服务器原始资料在/root/autodl-tmp/uav/records及logs。代码、构建和虚拟环境均在数据盘；关机保留与释放实例是两回事，保存系统镜像不会自动包含数据盘。记录已回存本地仓库，但本次没有制作完整数据备份或系统镜像，也没有自动提交或上传GitHub。


## 项目场景的后续验证

任务5至6已完成，日常启动优先使用tools/deployment/launch_project_rgbd.sh，不能把原来的标准Iris空场景命令当作项目完整启动入口。RGB、深度、CameraInfo和点云统一使用camera_link_optical，主点为319.5/239.5，详见rgbd_validation_2026-11-02.md。
