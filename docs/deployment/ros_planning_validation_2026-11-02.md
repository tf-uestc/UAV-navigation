# 2026年11月2日 ROS接口与EGO规划验证

本轮完成用户批准的“保存快照、第7项ROS接口与安装验证、第8项EGO规划链验证”。这些编号是本次部署计划的编号，不是原Ubuntu清单的U07和U08。验证范围到规划输出为止；没有启动执行器、解锁、起飞、OFFBOARD控制或收费视觉API。

## 快照和版本

- 实施分支：`fix/navigation-stability`，main保持不变。
- 修改前业务快照：`b4b442342da2bc0afee072f3a3043bfab5b86d4f`，包含任务1至6的已验证环境、模型、脚本和记录。
- EGO基线：`bfda51284c8c1b476043255a8145ef925a3778a5`；本轮修复提交：`f5f62661df07cdc88363a048ebfa16bc7b85d889`，分支`uav-integration`。
- PX4保持v1.14.0，提交`b8c541dd7277ed735139d7d1bfb829d61fbe29fb`；没有修改PX4控制器。
- 17个回存源文件的SHA256见本批`source-manifest.json`。原始记录和补丁均纳入业务仓库；完整Git bundle保存在仓库外，避免把约54 MiB的外部历史当成业务源文件提交。
- 本轮保存的是Git代码快照和复现记录，没有制作整块数据盘备份或系统镜像，也没有推送GitHub。

## 第7项 实际完成的修改

补齐catkin_python_setup，使navigation_core、planning和检测辅助模块可在开发及安装环境中导入；修正launch/config的安装目录，增加可安装的测试入口与实际运行依赖。验证从/tmp执行，避免当前源码目录掩盖安装缺口。

增加planning-only入口`planner_verified.launch`，只启动EGO、规划网关和相机位姿桥。任务请求、EGO接受确认、连续样条和取消确认均带会话、mission_id和goal_id；重启后旧会话、迟到请求和取消消息不能恢复旧任务，也不能取消较新的任务。

任务管理器在显式启用`~identified_planning`时使用新协议。该选项默认关闭，保留旧入口作为对照。任务状态改为保留最新状态，并在初始化后发布IDLE，解决刚连接的订阅者错过状态的问题。检测故障测试使用固定fixture和真实rospy序列化，不连接云端模型。

本地撤销轨迹权限会立即生效；EGO发回确认后才报告CANCELLED。未收到确认则有界报告CANCEL_TIMEOUT，不把本地等待结束误当作EGO已停。

## 第8项 实际完成的修改

EGO增加可选的managed_control接口以及GoalCommand、GoalStatus消息。扩展Bspline消息保留原始控制点、节点、阶数和起始时间，并增加会话及目标标识。消息MD5发生变化，必须同时重新编译EGO及全部消费者，不能混用上游原版消息。

受管模式保留请求高度，拒绝越界、已知占据或不合约定高度的目标。连续重规划及停车样条都经过目标归属检查后转发；取消使EGO清除目标并进入等待状态；恢复从实测状态重新规划。地图或里程计失效会停止旧目标，恢复数据不会自动恢复旧任务。

EGO入口将MAVROS里程计中的机体速度旋转到世界坐标；原MAVROS消息不变。独立点云输入在该launch中显式关闭，只使用深度图与同一采集时刻的光学相机Pose。相机桥从EGO参数命名空间读取同一套K，并检查实际CameraInfo、帧、数据年龄与历史TF；校准不匹配时不发布有效输入。

规划输出为`/uav/planning/trajectory`原始Bspline；`/uav/trajectory_preview`仅用于可视化。本入口不向旧飞行执行话题发布控制轨迹，也不启动follower、setpoint_bridge或解锁节点。

## 验证结果

| 验证 | 结果 | 证据 |
| --- | --- | --- |
| Python 3.8离线回归 | 81项，0失败、0错误 | offline-final.txt |
| 开发环境ROS生命周期与故障注入 | 13项通过，使用真实ROS时间 | lifecycle-devel-wall-pass/results.json |
| 安装环境标准rostest | 14项通过，含可控/clock暂停 | lifecycle-install-rostest-accepted/results.json及rostest-accepted.log |
| EGO安装环境真实C++规划 | 13项通过 | ego-install-verified/results.json |
| EGO开发环境真实C++规划 | 13项通过 | ego-devel-release/results.json |
| 安装后相机桥 | 15组图像/Pose同时间戳；历史TF位置误差0；错误K被拒绝 | camera-installed-validation.json |
| EGO补丁恢复 | 干净bfda512基线上应用补丁，15个修改文件逐字节相同 | ego-patch-restore.json |
| 收尾飞控状态 | connected=true、armed=false | final-live-state.json |

ROS生命周期测试覆盖独立检测ID与Header.seq不同、成功/失败/BUSY、快速响应、任务切换、迟到结果、地图就绪重试与超时、取消前后乱序、网关和任务管理器重启、旧latched消息与暂停时钟。所有等待以墙钟截止时间约束。

EGO测试覆盖四种航向下的速度变换、已知障碍在平移及90度转向后的占据位置、占据及越界目标拒绝、1.5/2.0/2.2米高度保持、静态绕障、动态障碍触发重规划、至少三条连续样条转发、取消确认、恢复起点、数据中断和EGO重启。Python在101个时刻与实际C++ UniformBspline的位置、速度、加速度对照；所选样本最大数值误差小于1e-12，验收阈值仍为1e-6，未放宽。

生命周期测试使用独立ROS master 11312，EGO测试使用11313，标准rostest父进程使用11314；没有把这些故障注入接入飞行master 11311。相机桥的单独只读测试使用现有场景数据。合成生命周期图像为16×12、50Hz以隔离协议，不能据此推断实际640×480相机吞吐或视觉准确率。

## 复现命令

先按external/ego-planner/README.md恢复固定EGO补丁并编译。ROS节点使用系统Python，不能激活PX4构建虚拟环境或Conda再构建业务包。

```bash
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/noetic/setup.bash
source /root/autodl-tmp/uav/ego-planner/install/setup.bash
cd /root/autodl-tmp/uav/UAV-navigation/ws
catkin_make --build build_verified -j4 -l4 install \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_DEVEL_PREFIX=/root/autodl-tmp/uav/validation/business-devel \
  -DCMAKE_INSTALL_PREFIX=/root/autodl-tmp/uav/validation/business-install
source /root/autodl-tmp/uav/validation/business-install/setup.bash
cd /tmp
ROS_MASTER_URI=http://127.0.0.1:11314 \
UAV_STAGE78_RESULTS=/root/autodl-tmp/uav/records/step78-repeat \
  rostest uav_vln_bringup lifecycle_integration.test
```

开发环境替换source为EGO/devel和validation/business-devel。完整测试驱动为安装包share/uav_vln_bringup/test/run_stage78.py，支持`--stage lifecycle|ego --clock wall|sim --out 路径`；EGO阶段额外传`--oracle 可执行文件`。oracle由同目录bspline_oracle.cpp及固定EGO的uniform_bspline.cpp构建，具体编译命令见下。

```bash
cd /root/autodl-tmp/uav/ego-planner
g++ -O2 -std=c++14 -I/usr/include/eigen3 -I/opt/ros/noetic/include \
  -I src/planner/bspline_opt/include \
  /root/autodl-tmp/uav/UAV-navigation/ws/src/uav_vln_bringup/test/bspline_oracle.cpp \
  src/planner/bspline_opt/src/uniform_bspline.cpp \
  -L/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lcpp_common \
  -o /root/autodl-tmp/uav/validation/bspline_oracle
```

先确认11312至11314未被其他作业占用。驱动只清理自身创建的进程；不要关闭现有项目仿真来释放测试端口。已配置过的build_verified可用`cmake --build build_verified --target install -- -j4`保留缓存；重新catkin_make时应重复写明两个输出prefix，避免误装到ws/install。

## 原清单状态与未覆盖范围

U00、U01和U02现已完成并勾选。U03增加速度与相机桥证据，但运动时定位和真实检测延迟仍未覆盖。U04增加单输入建图、障碍变换和新鲜度证据，但未知空间策略、任务围栏与全局安全空间仍未统一。U05的目标确认、连续样条、规划侧停止恢复已通过，实际唯一执行器接受反馈和运动连续性尚未完成，因此整项保留未勾选。

旧vlm_navigation_full.launch、旧EgoPlannerAdapter和原飞控接线没有自动切换为本轮新入口。下一步应先完成唯一执行器和停止反馈，再做基础SITL起降、航向保持及实际避障，最后接真实检测和分段接近闭环。当前测试不证明实际制动距离、飞行避障、云端检测准确率或到达后的180度旋转已经解决。

原始记录目录同时保留排查中的失败日志。上表列出的accepted/verified/pass目录才用于本次通过结论，不能把历史失败目录当成最终结果，或反过来删除它们掩盖排查过程。
