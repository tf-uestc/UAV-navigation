# F3 — Plan: TF 链实现方案

## 1. 总览

不写新节点，**只改三处配置**：
1. mavros launch 启用 TF 发布 + `map` frame
2. SDF（或 launch）补齐相机静态变换
3. F2/F4 用 `tf2_buffer.transform()` 替代 yaw-only 近似

外加一个**TF 健康检查脚本** + 一个**动捕接入 launch 模板**。

## 2. 目标 TF 树

```
   map
    │  (mavros: PX4 EKF 输出, 仿真里=Gazebo 真值, 动捕里=动捕原点)
    │
   odom            (短期平滑; 仿真里直接 = map)
    │
   base_link       (FLU, 机体)
    │  (静态: SDF 里相机的安装偏移; 例如 iris_depth_camera 的 0.1m 前/0.05m 下)
    │
   camera_link     (FLU, 相机机械原点)
    │  (静态: ROS 标准旋转 R = R_z(-90°) * R_x(-90°))
    │
   camera_link_optical   (z→前, x→右, y→下; 图像 frame_id 用这个)
```

## 3. 第一处：mavros 配置

`mavros_px4_sitl.launch` 现在直接 include 了 `mavros/launch/px4.launch`。需要确认下面参数（在 mavros 的 `px4_config.yaml` 里）：

```yaml
local_position:
  frame_id:        "map"          # 输出 PoseStamped 的 frame
  tf:
    send:          true            # 关键: 让 mavros 发 map→base_link TF
    frame_id:      "map"
    child_frame_id: "base_link"
```

**改动**：在 `ws/src/uav_vln_bringup/config/` 下放一份 override 的 yaml，在 `mavros_px4_sitl.launch` 里通过 `<rosparam file=".../px4_pluginlists.yaml">` 装载，覆盖默认。

> 默认 mavros 不一定开 TF 发布，开了之后 `rqt_tf_tree` 立刻能看到 `map → base_link`。

## 4. 第二处：相机静态 TF

两条路径，**先看 SDF 后再决定**：

### 路径 A：SDF 已经在发

打开 `simulation/px4_gazebo_classic/models/iris_depth_camera/iris_depth_camera.sdf`（README 里提到，但仓库目前没有这个目录——大概率在 `$PX4_AUTOPILOT_DIR/Tools/sitl_gazebo/models/iris_depth_camera/` 里），找到相机插件部分：

```xml
<plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
  <frameName>camera_link_optical</frameName>   <!-- 这里 -->
  ...
</plugin>
```

如果 `<frameName>` 已经是 `camera_link_optical`，配合 `<gazebo reference="camera_link">` 块里的 `<sensor>` 位姿，TF 就齐了，**不用动**。

### 路径 B：SDF 没发或 frame 名不一致

在 launch 里补一段 `static_transform_publisher`：

```xml
<!-- base_link → camera_link: 相机安装偏移(按 SDF 实际值改) -->
<node pkg="tf2_ros" type="static_transform_publisher"
      name="cam_mount_tf"
      args="0.10 0.0 -0.05 0 0 0 base_link camera_link"/>

<!-- camera_link → camera_link_optical: ROS 光学系标准旋转 -->
<!-- (yaw=-pi/2, pitch=0, roll=-pi/2) -->
<node pkg="tf2_ros" type="static_transform_publisher"
      name="cam_optical_tf"
      args="0 0 0 -1.5708 0 -1.5708 camera_link camera_link_optical"/>
```

**最稳的诊断顺序**：先 `rosrun rqt_tf_tree rqt_tf_tree` 看树，缺哪段补哪段。

## 5. 第三处：删除 yaw-only 近似

把 `vlm_navigation.py:267-293` `camera_to_world_approx` 整个函数删掉。所有调用点（在 `_run_vlm_cycle` 里）改成：

```python
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# 节点 __init__ 里:
self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

# 用的时候:
def camera_point_to_map(self, x_c, y_c, z_c, stamp) -> Optional[Tuple[float, float, float]]:
    pt = PointStamped()
    pt.header.frame_id = "camera_link_optical"
    pt.header.stamp = stamp     # 用图像 header.stamp,不是 now()
    pt.point.x, pt.point.y, pt.point.z = x_c, y_c, z_c
    try:
        out = self.tf_buffer.transform(pt, "map", rospy.Duration(0.2))
        return (out.point.x, out.point.y, out.point.z)
    except (tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException) as e:
        rospy.logwarn_throttle(2.0, "[TF] transform failed: %s", e)
        return None
```

> 用图像的 `header.stamp` 而不是 `rospy.Time.now()` 是关键——这样 tf2 会查"那一瞬间的飞机姿态"，避免 50ms 网络延迟带来的姿态错位。

## 6. 动捕接入预案（不在 F3 实现期做，但 launch 写好）

新建 `ws/src/uav_vln_bringup/launch/mavros_mocap.launch`：

```xml
<launch>
  <arg name="rb_name" default="iris"/>

  <!-- VRPN client 拉动捕系统位姿 -->
  <include file="$(find vrpn_client_ros)/launch/sample.launch">
    <arg name="server" value="192.168.1.100"/>
  </include>

  <!-- remap 到 mavros vision_pose, PX4 EKF 会融合 -->
  <node pkg="topic_tools" type="relay" name="mocap_to_vision"
        args="/vrpn_client_node/$(arg rb_name)/pose
              /mavros/vision_pose/pose"/>

  <!-- mavros 主 launch -->
  <include file="$(find uav_vln_bringup)/launch/mavros_px4_sitl.launch">
    <arg name="fcu_url" value="/dev/ttyUSB0:921600"/>   <!-- 真机串口 -->
  </include>
</launch>
```

PX4 EKF 融合后，`/mavros/local_position/pose` 仍然按 mavros 配置发布到 `map`，**对 F2/F4 完全透明**。

## 7. TF 健康检查脚本

`ws/src/uav_vln_bringup/scripts/tf_healthcheck.py`：启动时检查关键变换是否存在：

```python
required = [
    ("map", "base_link"),
    ("base_link", "camera_link_optical"),
    ("map", "camera_link_optical"),
]
for parent, child in required:
    try:
        buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(5.0))
        rospy.loginfo("[TF] OK: %s -> %s", parent, child)
    except Exception as e:
        rospy.logerr("[TF] MISSING: %s -> %s (%s)", parent, child, e)
        sys.exit(1)
```

放到 `vlm_navigation_v2.launch` 顶部当 `<node ... required="true">` 跑，TF 没准备好就别让下游节点起来。

## 8. 取舍

| 决策 | 选择 | 备选 | 理由 |
|---|---|---|---|
| 实现层级 | 配置 + tf2_ros 标准 API | 写自定义 frame manager | ROS 已经把这事解决了，自写没必要 |
| 时间戳 | 用图像 stamp | `Time(0)`（最新） | 关联到具体图像帧才能保证姿态一致 |
| 动捕路径 | 预留 launch + relay | 现在就接入 | 没硬件，不烧时间 |
| odom 中间层 | 保留（mavros 默认有） | 直接 map → base_link | 留一层让真机 VIO/动捕路径可以替换 odom 实现 |

## 9. 风险

- **frame_id 在不同地方拼写不一致** 是这类 bug 最常见来源（`camera_link` vs `camera_link_optical` vs `iris/camera_link`）。tf_healthcheck.py 一定要写。
- **mavros TF 发布开关默认关闭**：第一次启动看不到 `map → base_link`，先怀疑这里。
- **动捕路径 PX4 参数**：真接入时 `EKF2_AID_MASK` 要打开 vision position fusion，`EKF2_HGT_MODE` 改成 vision，否则 PX4 不信动捕数据。
