# U11 原始 28 项与固定版本恢复核对

11 月 25 日 10:59（北京时间）只读截取了候选 `3b4412a27fe996a7dd73946c65c645ddd8a4c2f3` 的 28 份完整仿真启动记录及第 29 次尚未写全的记录。28 轮都从已着地、未解锁且 MAVROS 已连接的状态开始，源码状态均为空；每轮 `start.json` 的 `reused` 均为 false。PX4、Gazebo、MAVROS 各有 28 个不同的 PID，EGO、飞行执行器和任务节点的启动 PID 也逐轮记录。此处的 28 轮指 22 轮规划和已完成的 6 轮云端飞行；下表的 28 项指 10 月 27 日的架构审查问题。后续飞行结果以最终逐轮报告为准。[逐轮启动与实际参数快照](u11-restore-snapshot.json) · [原审查](../../../reviews/architecture_audit_2026-10-27.txt) · [历史回核](../../audit_followup_2026-11-09.md)

## 恢复了什么

- **业务代码与 EGO。** 独立构建目录 `install03` 从上述业务提交和 EGO `bfda512` 恢复；业务源码覆盖清单为空。EGO 使用归档的四份补丁。安装检查定位到该目录自己的 `ego-install/share/ego_planner` 和 `business-install/share/uav_vln_bringup`。系统 Python 3.8、NumPy 1.17.4、OpenCV 4.2.0，Gazebo Classic 11.15.1、MAVROS 1.20.1。候选在 Ubuntu 上的离线回归为 125/125。[版本、补丁及参数哈希](u11-version-manifest.json) · [离线报告](candidate03-offline.log)
- **PX4。** 本次运行实际使用 11 月 9 日从源码重建的 PX4 SITL 及 Gazebo 插件。今日核验源码标签 `v1.14.0`、提交 `b8c541d`、30 个递归子模块及跟踪源码状态。归档可执行文件、MAVLink 及电机插件、实际加载的 Iris 深度相机 SDF、项目世界的 SHA256。原 U00 要求固定 PX4 及子模块，U01 明确要求分别编译 EGO 与业务包，U11 要求从固定提交在干净构建/安装目录恢复。原清单没有要求每次重编相同 PX4 依赖。今日恢复结论覆盖同一 Ubuntu 依赖环境内新建的业务/EGO 安装目录和受版本控制的 PX4 依赖；跨镜像从零安装另需实验。[原清单](../../../reviews/ubuntu_tasks_2026-10-29.txt) · [11 月 9 日源码构建](../../final_regression_2026-11-09.md) · [今日指纹](u11-version-manifest.json)
- **业务参数与发布权。** 活动 ROS 参数显示 `require_known_space=true`、独立点云输入关闭、地图网格 0.1 米、尺寸 40×40×3.5 米、下界 −0.5 米、障碍膨胀 0.349 米。任务围栏 x 为 ±6 米、y 为 ±4 米、z 为 1.1–2.35 米。速度/加速度上限 0.32 m/s 和 0.35 m/s²。采样时的任务高度 1.6 米、目标停距 2.4 米，内部停距容差 0.12 米。ROS master 在采样时返回 `/uav/goal_pose` 和 `/mavros/setpoint_position/local` 的唯一发布者均为 `/flight_executor`。完成轮次 F03-2-straight 的启动参数与上述值相符。[只读 ROS 与启动快照](u11-restore-snapshot.json)
- **重启的复验效果。** F03 五次直线分别重新启动并通过独立验收。22 轮初始自动复核 20/22，经原始物理接触流及落地后采样边界独立复核后为 21/22，剩余失败为 F03-4-long 的航向夹具超时。16 轮云端任务的最终结论须由完整飞行矩阵给出。重启证据、规划结果与云端任务结果分别计数。[规划原始物理复核](planning-final/audit/F03-initial-22.json) · [失败轮夹具回放](planning-final/heading-fixture/F03-4-long-replay.json)

## 原问题逐项复核

| 编号及问题 | 今日证据、历史边界和待补之处 |
|---|---|
| 01 样条几何、时间被改写 | 真实 EGO 的 101 个时刻与 C++ 数值解对照通过；正式动态回合检查了实际执行的已接受轨迹。[数值对照](ego-a1/native/results.json) · [动态回合](planning-final/audit/reports/F03-1-dynamic.json) |
| 02 连续重规划与急停丢失 | 今日检查同目标连续样条、逐 ID 接受/拒绝、迟到旧轨迹、障碍可绕和无路实停。动态通过样本目前只代表这些具体箱体场景。[交接夹具](ego-a1/handover.json) · [动态可绕](planning-final/audit/reports/F03-1-dynamic.json) · [无路](planning-final/audit/reports/F03-1-blocked.json) |
| 03 轨迹缓存时间戳、末端回跳 | 今日跑过已有轨迹不可变、末端保持和时钟回退离线用例。[候选离线清单](candidate03-offline.log) |
| 04 逐点提前停距吞掉短指令 | 历史基础飞控已专测 0.3 米轨迹点，今日五轮短直线均有 Gazebo 终点及轨迹证据。两者测试距离不同。[历史飞控](../../flight_execution_validation_2026-11-09.md) · [今日规划矩阵](planning-final/audit/F03-initial-22.json) |
| 05 意外航向归零 | 历史 SITL 有旧入口复现及航向连续性对照，今日长程夹具一轮失败已回放。该回放显示夹具用旧位置生成到达距离；它不能证明在任意姿态都不会发生异常旋转。[历史专项](../../heading_validation_2026-11-09.md) · [今日回放](planning-final/heading-fixture/F03-4-long-replay.json) |
| 06 悬停状态与实际停稳 | 候选 ROS 假飞控故障注入证实主动悬停消息丢失后仍能独立撤销旧轨迹。正式无路回合另有 Gazebo 前后的真实停止检查；云端回合需单列完整十个仿真秒。[隔离试验](flight-safety/flight-safety-attempt02.json) · [无路实际停止](planning-final/audit/reports/F03-1-blocked.json) |
| 07 暂停后旧 EGO 轨迹仍有效 | 今日已验从实测位置恢复，起点误差约 0.000172 米；反馈链断开后迟到样条在假飞控中引起的位移为 0 米。[真实 EGO](ego-a1/native/results.json) · [隔离执行器](flight-safety/flight-safety-attempt02.json) |
| 08 Header.seq 充当业务 ID | 隔离 ROS 实际序列化检查通过，任务号/请求号可独立匹配。[生命周期](lifecycle-sim-02-03/results.json) |
| 09 搜索接受其他任务的结果 | A→B、错误任务、重启后同编号旧结果拒绝通过。生命周期组验证既有消息契约，正式搜索策略另看逐轮日志。[生命周期](lifecycle-sim-02-03/results.json) |
| 10 规划不就绪无限等待 | 永久未就绪及规划超时有界退出通过；正式飞行无路可停止。[生命周期](lifecycle-sim-02-03/results.json) · [无路飞行](planning-final/audit/reports/F03-1-blocked.json) |
| 11 建图双输入与坐标混用 | 正式参数关闭独立点云，使用深度和相机位姿；真实 EGO 移动相机、转 90° 后固定障碍仍在世界位置。[实际参数](u11-restore-snapshot.json) · [EGO 地图](ego-a1/native/results.json) |
| 12 机体速度错作世界速度 | 今日四航向的 1 m/s 数学对照通过，历史飞行还有独立 MAVROS 世界速度对照。[EGO 原始误差](ego-a1/native/results.json) · [U03 证据](u03-review.md) |
| 13 起飞阻塞、取消与超时误报 | 旧版假飞控上升时取消直接解除解锁；候选在 0.224 米取消，着地标志冻结时走降落流程，唯一上锁在 0 米。历史 SITL 另有起飞异常矩阵。[旧版](flight-safety/flight-safety-before-attempt02.json) · [候选](flight-safety/flight-safety-attempt02.json) · [历史实际飞控](../../flight_execution_validation_2026-11-09.md) |
| 14 悬停锚点漂移、恢复受阻 | 今日离线固定锚点及 EGO 撤销后恢复用例通过；11 月 23 日八次到达的悬停测量时长不足。最终云端逐轮需核查十个仿真秒。[离线](candidate03-offline.log) · [旧时钟窗口回放](hold-clock-before.json) |
| 15 RGB/深度不同步、快照过期 | 今日真实检测节点受控回放拒绝深度错时、内参帧不匹配和过期回复。[感知回放](u03-replay-final.json) |
| 16 缺失历史 TF 时错用当前位姿 | 今日延迟响应使用拍摄时 TF，缺 TF 明确拒绝；实际高度误差另有独立记录。[回放](u03-replay-final.json) · [运动真值](u03-height-analysis.md) |
| 17 模型框单位不明确 | 原生标签、框格式与错单位拒绝均有回放。真实模型仍可能对空场景返回合法框。[响应协议](u03-replay-final.json) · [误识别逐帧复核](right90-review/findings.md) |
| 18 目标估计及异常观测门控 | 对历史两次右 90° 退出逐帧复算，六个误报框在三种替代深度下均被颜色证据拒绝。原深度图缺失，未据此推断当时每个像素深度。最终成功率以云端实飞为准。[专项](right90-review/findings.md) |
| 19 快速回调丢失 | 真实 ROS 快速 BUSY 和等待清理通过；为既有管理器接口测试。[生命周期](lifecycle-sim-02-03/results.json) |
| 20 多个控制发布者争用 | 今日正式 ROS master 快照显示两条设定值话题均为唯一执行器发布；历史有冲突注入。当前只审查正在使用的分段入口。[实际发布者](u11-restore-snapshot.json) · [历史飞控](../../flight_execution_validation_2026-11-09.md) |
| 21 刹停和转稳前拍照、搜索时限 | 今日专项验证回包时仍在运动会要求重新停稳，旧记录存在云端误报；五方向任务逐轮需检查停稳、超时及最终真值。[到达判定](visual-arrival-depth-after.txt) · [历史误报](right90-review/findings.md) |
| 22 旧规划成功与当前请求混淆 | 今日核对样条归属、同任务及目标 ID 和迟到撤销；动态取证另检查执行轨迹属于已接受 ID。[生命周期](lifecycle-sim-02-03/results.json) · [动态实际接受关系](planning-final/audit/reports/F03-1-dynamic.json) |
| 23 已取消目标复活 | 真实 EGO 取消和节点重启后拒绝旧样条，单向反馈中断有执行端撤销证据。[EGO 重启](ego-a1/native/results.json) · [反馈断开](flight-safety/flight-safety-attempt02.json) |
| 24 搜索平移绕过 EGO | 旧版有搜索平移实际 EGO 记录；今日纯规划覆盖实际移动和停止，无搜索航点文字任务的转向顺序在云端全矩阵单列。[历史搜索](../../final_regression_2026-11-09.md) · [今日规划](planning-final/audit/F03-initial-22.json) |
| 25 物体位置、目标高度、停距混淆 | 今日 EGO 的 1.5、2.0、2.2 米规划目标高度保留；过近或仍在动不报到达。历史式“2.4 米基本成功”不能代替本日 1.5 米六次飞行。[EGO 高度](ego-a1/native/results.json) · [本地到达专项](visual-arrival-depth-after.txt) |
| 26 就绪、边界、相机标定 | 今日已有实际边界与未知区保护参数、地图障碍坐标法和已知点多姿态重建。运动时机体高度估计与 Gazebo 真值仍有偏差，历史/今日纯规划结果均限定于所测空间。[实际参数](u11-restore-snapshot.json) · [地图](ego-a1/native/results.json) · [相机绝对高度](u03-height-analysis.md) |
| 27 飞控状态反馈与输入保护 | 今日假飞控验证着地矛盾、断反馈撤销；正式规划逐轮另核对上锁、停稳及意外接触。实际飞行失联故障的历史依据可查基础飞控矩阵。[假飞控](flight-safety/flight-safety-attempt02.json) · [正式规划](planning-final/audit/F03-initial-22.json) · [历史飞控](../../flight_execution_validation_2026-11-09.md) |
| 28 环境复现与完整回归 | 同一固定业务源码重新构建到独立目录，重复全模拟栈冷启动已证实。U11 总结还需纳入最新云端完整矩阵、任何后续候选差异、重试次数和失败原因。[运行快照](u11-restore-snapshot.json) · [版本指纹](u11-version-manifest.json) |

若测完后采用另一候选，应比较 EGO/业务控制源码、配置和试验驱动的差异。影响飞行的差异需要在该版本留存对应复测。若只改验收算法，保留旧判定并对同一原始记录重新计算。所有轮次应同时留有运行退出状态和检查结论，尤其保留最初失败轮。

11 月 25 日最终复核：冻结版 `6b58825` 的 39 轮冷启动有独立启动记录，包含 23 轮规划与 16 轮真实云端。云端按原验收 14/16，同时要求执行器正常结束为 13/16。1.5 米左 45° 的第二轮物理着地且已上锁，旧执行器仍报告超时。分支 `1581635` 的四轮无云端现场补验均看到物理着地、执行器正常结束及任务 LANDED。[39 轮恢复证据](u11-f04-v04-startup.json) · [最终逐轮结论](u11-flights-V04.md) · [降落修复补验](landing-followup/README.md)
