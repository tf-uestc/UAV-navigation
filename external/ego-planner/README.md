# EGO规划接口补丁

业务代码的planner_verified.launch要求此固定版本，不兼容未扩展消息的原版EGO。

- 上游：https://github.com/ZJU-FAST-Lab/ego-planner.git
- 基线：bfda51284c8c1b476043255a8145ef925a3778a5
- 修复提交：f5f62661df07cdc88363a048ebfa16bc7b85d889
- 补丁：managed-integration.patch，包含Noetic顶层catkin链接、安装规则、受管目标/停止接口、消息和地图入口修正。
- 完整历史bundle：ego-managed-integration-2026-11-02.bundle，保存在本地deployment备份目录及服务器/root/autodl-tmp/uav/records，不纳入本业务仓库。

在干净副本恢复，不要在已经修改过的EGO上重复叠加：

```bash
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git ego-planner
cd ego-planner
git switch -c uav-integration bfda51284c8c1b476043255a8145ef925a3778a5
git apply --check ../UAV-navigation/external/ego-planner/managed-integration.patch
git apply ../UAV-navigation/external/ego-planner/managed-integration.patch
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4 -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make -j4 -l4 install -DPYTHON_EXECUTABLE=/usr/bin/python3
```

也可直接git clone完整bundle后切换uav-integration；这种方式保留f5f6266提交本身。补丁方式生成相同文件，但自行提交时提交哈希不一定相同。干净基线恢复与15个修改文件内容一致性已经验证，见本轮ego-patch-restore.json。

Bspline新增planner_session_id、mission_id、goal_id和emergency_stop，ROS消息MD5已改变。EGO和全部业务消费者都要重编译、重新source，禁止同时混用上游旧消息和本补丁消息。managed_control默认关闭；本项目的planner_verified.launch明确开启，且只做规划，不启动飞行。

详细复现与已验证边界见../../docs/deployment/ros_planning_validation_2026-11-02.md。
