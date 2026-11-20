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

## 已观测空间保护

使用 `segment_verified.launch` 时，先在受管 EGO 版本上应用 `known-space-guard.patch`，再重新构建并加载 EGO 和业务工作区。由 `planner_verified.launch` 直接启动规划测试时，可显式传入 `require_known_space:=true`。

```bash
git apply --check ../UAV-navigation/external/ego-planner/known-space-guard.patch
git apply ../UAV-navigation/external/ego-planner/known-space-guard.patch
source /opt/ros/noetic/setup.bash
catkin_make install -j4 -l5 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

规划网关只有在当前 EGO 会话报告保护已启用时才授权轨迹。新样条和重规划样条会检查已知空间和膨胀占据。每条样条固定使用授权时的实测起点。起点所在栅格及半个栅格内的边界抖动（当前至多 5 厘米）可作为机体已经占用的起点，膨胀占据检查仍然生效。其余新进入的采样点须经过观测。活跃样条也会复查未知空间。无法确认时任务停稳观测并最多重新规划一次。验收范围为项目 Gazebo SITL。

## November 20 acceptance candidate

Map readiness now requires a recent completed depth fusion. Out-of-bounds
poses, invalid depth, wrong frames and paused fusion cannot refresh readiness.
The verified Iris launch uses a map volume from z=-0.5 to z=3.0 in MAVROS map
coordinates; the flight fence remains unchanged.

The verified Iris base collision box is 0.47 x 0.47 x 0.11 m in iris.sdf.
UNKNOWN samples inside the current start body volume (and half a voxel for
discretization) are handled as the sensor's self-occlusion region. This
volume is anchored once per accepted trajectory. It never exempts inflated
occupancy, does not expand with trajectory progress, and does not authorize
an unobserved endpoint outside the initial body volume.

Only a singleton array containing one fully specified detection object is
normalized to that object. Multiple objects, bare coordinate arrays, missing
units, wrong units and invalid coordinates remain rejected.
