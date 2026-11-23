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

本节前述标准Iris数据对应先前任务1至4。随后用户批准任务5至6，并确认使用search_rescue.world；项目iris_depth_camera及RGB-D静态TF验证现已通过，详细修正和证据见rgbd_validation_2026-11-02.md。U00环境与完整模型基线已通过；后续第7至8项已完成U01/U02和EGO规划侧验证，详见本文末补充；U03动态实测、唯一控制出口、真实检测API、起飞和180度旋转仍待验证。

## 记录位置与备份

仓库docs/deployment/environment_2026-11-02.md是可版本管理的环境说明；同目录records/2026-11-02保存验证JSON、构建/运行日志、环境锁定清单和截图。docs/阶段性总结/阶段总结.docx保留阶段结论，不替代详细复现记录。

服务器原始资料在/root/autodl-tmp/uav/records及logs。代码、构建和虚拟环境均在数据盘；关机保留与释放实例是两回事，保存系统镜像不会自动包含数据盘。记录已回存本地仓库，但本次没有制作完整数据备份或系统镜像，也没有自动提交或上传GitHub。


## 项目场景的后续验证

任务5至6已完成，日常启动优先使用tools/deployment/launch_project_rgbd.sh，不能把原来的标准Iris空场景命令当作项目完整启动入口。RGB、深度、CameraInfo和点云统一使用camera_link_optical，主点为319.5/239.5，详见rgbd_validation_2026-11-02.md。


## 第7至8项后的环境与源码记录

EGO当前为uav-integration分支f5f62661df07cdc88363a048ebfa16bc7b85d889，基线bfda512。补丁见external/ego-planner；GoalCommand/GoalStatus和Bspline消息变更要求重编译全部消费者。PX4版本及专用构建venv保持原样，没有安装或升级CUDA、cuDNN、PyTorch。

ROS、相机桥、任务管理和检测fixture均使用/usr/bin/python3。推荐验证环境source顺序：/opt/ros/noetic/setup.bash → /root/autodl-tmp/uav/ego-planner/install/setup.bash → /root/autodl-tmp/uav/validation/business-install/setup.bash。开发环境对应ego-planner/devel和validation/business-devel，不能交叉混用旧消息。PX4构建继续单独使用venvs/px4-py38；离线测试使用venvs/offline-py38，81项回归通过。Conda不参与ROS构建。

业务验证构建目录为ws/build_verified，输出prefix明确为validation/business-devel和validation/business-install；旧ws/devel保留作为日常开发目录。完整命令、独立测试端口、相机标定共享方式和边界见ros_planning_validation_2026-11-02.md。原来的vlm_navigation_full.launch没有自动切换到新规划入口。

修改前业务快照b4b4423已制作完整Git bundle；本轮EGO提交也已制作bundle并核对SHA256。最终代码和记录另存同批修复分支提交及bundle；这不等于完整数据盘备份或系统镜像。本轮没有推送GitHub，main未合并。


## 2026年11月20日隔离环境与恢复状态

此前各日期记录作为历史保留；本节记录本轮实际使用的隔离前缀。八项扩展任务的阶段结果和未通过条件见[本轮实施进度](final_tasks_2026-11-20.md)。最终真实云端回归未通过，实例已由用户关闭，候选未作为新版本发布。

| 使用场景 | 解释器及依赖 | 边界 |
| --- | --- | --- |
| ROS、相机、任务与当前云端检测 | /usr/bin/python3；Python3.8.10，NumPy1.17.4，OpenCV4.2.0 | 不激活PX4或Conda环境运行ROS节点 |
| PX4构建 | /root/autodl-tmp/uav/venvs/px4-py38/bin/python；NumPy1.23.5、EmPy3.3.4 | 仅供构建依赖使用，未写入系统环境 |
| 离线测试 | /root/autodl-tmp/uav/venvs/offline-py38/bin/python；NumPy1.24.4 | 独立测试，不替代ROS运行解释器 |

PX4固定v1.14.0、b8c541dd7277ed735139d7d1bfb829d61fbe29fb。新冷恢复目录为/root/autodl-tmp/uav/validation/restore-final-2026-11-20-jXaGiA，依次加载/opt/ros/noetic/setup.bash、该目录ego-install/setup.bash、business-install/setup.bash，不混用历史devel或其他前缀。启动脚本在子进程中清理Python与catkin路径，原Conda环境保留。

EGO按上游bfda51284c8c1b476043255a8145ef925a3778a5加managed-integration、known-space-guard、dependency-manifest三份补丁恢复。最后一份补齐geometry_msgs、quadrotor_msgs、cv_bridge和message_generation依赖声明。新工作区重建与两次重启恢复通过；固定日常入口及全部源码补丁回存尚待最终收尾。

相机机型为iris_depth_camera，使用项目search_rescue.world。模型为qwen3.8-omni-flash，仍调用云端接口，未增加本地视觉模型或改装CUDA/PyTorch。私有密钥由/root/autodl-tmp/uav/UAV-navigation/.env提供，不复制到文档、Git或普通备份。单次连接/读取超时3/35秒，检测截止和结果最大年龄42秒；任务总预算最多12次检测、720秒，搜索与重试共同消耗预算。

恢复脚本check/start/session重复调用及两次restart已验收，默认不解锁；拒绝在armed或落地状态未确认时stop。完整命令和固定入口仍需随最终通过版本归档。当前安装前缀包含尚未通过负样本回归的格式适配候选；开机后先保持未解锁，只读核验及修复感知，不能直接把该候选当作正式任务版本。

原始记录位于/root/autodl-tmp/uav/records/final-tasks-2026-11-20-o5wM。最新98项离线测试通过不代表云端定位通过：12次观测中2次误识别仍待修复。GitHub仍是836a385，实现2.4米；本轮源码完整回存、最终验收和推送均未完成。阶段总结记录结论及下一步，详细复现留在部署文档，失败证据不删除。


## 2026年11月23日固定入口与复验结果

日常入口已固定为/root/autodl-tmp/uav/runtime/uav.sh，同目录runtime.env记录发布源码位置、EGO和业务安装前缀、固定PX4及私有密钥文件路径。入口模板为tools/deployment/uav.sh；check/status/vnc/start/session不会自动解锁。已在/tmp以外部入口检查环境，VNC本地端口16006至服务器6006握手通过。

恢复可依次运行入口的check、vnc、start、session --height 1.6 --standoff 2.4；查看使用status，降落使用land。stop/restart要求落地且解除解锁。GUI保留供用户VNC观察。实际发布源码和运行时哈希在本轮备份及发布清单中核对，未将Conda或PX4虚拟环境依赖混入ROS。

本轮格式修复通过106项离线、21项ROS、6项重试搜索、63条历史回复及12次固定图像API检查。随后10轮实际云端任务用60次API调用，8轮到达但只有3轮通过真值悬停等最终验收，全部降落上锁且无意外场景接触。五方向未全通过，1.5米未执行。里程计高度与世界真值存在显著变化差异，未宣称已解决；误识别和可见性按用户要求暂缓。

完整本地备份在.local-backups/2026-11-23，Git忽略这些大文件，只收录精简证据。报告见[五方向复验与收尾](workflow_closeout_2026-11-23.md)。修复分支保留未通过条件，不自动合并main，不把本次保存解释为系统完整验收通过。
