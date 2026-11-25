# 11 月 25 日仿真试验与证据工具

23 轮规划试验由同一进程按顺序控制 Gazebo 中的 Iris。每次试验使用独立编号、目录和一套新启动的 PX4、Gazebo、MAVROS。长航距和中途加箱的补试在原批结束并再次确认落地、上锁后进行。每个编号关联业务提交、ROS 轨迹、物理接触及落地检查。

| 文件 | 本次用途 |
|---|---|
| `f04_matrix.py` | 在隔离安装中调度规划试验，保存主程序退出状态、独立核验、前后两次落地检查和每轮结果。保留原始判定以便复盘。 |
| `f04_unknown_observer.py` | 向未知区专项添加只读 Gazebo 接触观察器。 |
| `f04_long_retry.py`、`f04_obstacle_retry.py` | 针对已登记失败的场景执行独立编号补试，沿用原来的版本、场景与判据。 |
| `f04_targeted_checks.py` | 使用单独编号比较窄箱横向平移 0.30 米后的导航，随后复测未知区输入恢复。保存原始终态判断与按任务及降落指令顺序核查的结果。 |
| `verify_planning.py`、`verify_grounded_shutdown.py` | 按 Gazebo 原始物理帧核验出现过真实接触、飞行采集区间连续、意外接触计数，以及解除解锁时的记录边界。 |
| `collect_f04_v04_restore.py` | 从逐轮启动证据核对安装版本、进程、节点、停距和高度配置。 |
| `collect_cloud_receipts.py` | 核对真实模型调用日志。导出行号、模型和摘要，用于比对两批 API 调用数。 |
| `archive_batch.py` | 从完成的流水中选取图像、深度、运动、物理接触等原始证据，生成归档摘要。 |
| `verify_u04_final_install.sh` | 在独立 ROS master 下对四个起始航向及地图输入失效与恢复重复测试。 |
| `build_landing_candidate.sh`、`run_landing_retests.py`、`observe_landing.py` | 构建降落修复的独立候选并依次跑四轮，接收 PX4、里程计、执行器和任务层的逐条实际反馈。 |
| `report_landing_trials.py` | 接受本地大文件归档目录为参数，核对 T05 摘要并重算上锁至软件完成期间的速度样本及时间顺序。 |
| `restore_final_standby.sh` | 匹配本次实例的固定安装、复验归档和落地状态，恢复云端受测版本至空闲并核对唯一控制节点。 |

完整视觉任务入口在受测提交的 `tools/deployment/run_visual_matrix.py`。所有飞行脚本均用于本次 AutoDL 的专用 Gazebo 实例；各轮原始日志及编号用于区分实际动作、自动判定与独立复核结论。
