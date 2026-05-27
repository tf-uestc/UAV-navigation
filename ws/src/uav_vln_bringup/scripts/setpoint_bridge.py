#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
setpoint_bridge.py - 飞控接口中间层节点

【核心设计理念】
将 MAVROS/PX4 底层交互（OFFBOARD、arm、setpoint 流、安全策略）
封装在此节点中，算法层只需向 /uav/goal_pose 发布目标位姿即可。

【订阅话题】
  /uav/goal_pose     (PoseStamped)  - 算法层给出的目标位置
  /uav/state_cmd     (String)       - 控制命令: takeoff / goto / hover / land / rtl
  /mavros/state      (mavros_msgs/State)
  /mavros/local_position/pose  (PoseStamped)

【发布话题】
  /mavros/setpoint_position/local (PoseStamped) - 20Hz 连续 setpoint 流
  /uav/bridge_status  (String) - 当前状态机状态

【参数】
  ~setpoint_rate   (int, default: 20)    setpoint 发布频率 (Hz)
  ~takeoff_alt     (float, default: 2.0)  默认起飞高度 (m)
  ~goal_timeout    (float, default: 2.0)  目标超时秒数,超时自动悬停
  ~approach_dist   (float, default: 1.0)  最终接近目标的前停距离 (m)
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler


class BridgeState:
    """状态机枚举"""
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    FLYING = "FLYING"          # 正在飞往目标点
    HOVER = "HOVER"            # 悬停等待
    LAND = "LAND"              # 降落
    RTL = "RTL"                # 返航
    EMERGENCY = "EMERGENCY"    # 紧急停止
    COMPLETE = "COMPLETE"      # 完成


class SetpointBridge:
    def __init__(self):
        rospy.loginfo("[Bridge] Initializing SetpointBridge...")

        # ====== 参数 ======
        self.rate_hz = rospy.get_param("~setpoint_rate", 20)
        self.takeoff_alt = rospy.get_param("~takeoff_alt", 2.0)
        self.goal_timeout = rospy.get_param("~goal_timeout", 2.0)
        self.approach_dist = rospy.get_param("~approach_dist", 1.0)

        # ====== 状态 ======
        self.state = State()                    # MAVROS 的 FCU 状态
        self.current_pose = PoseStamped()       # 当前位置
        self.goal_pose = None                   # 算法层最新目标 (None=无目标)
        self.pending_goal = None                # 待处理的目标 (暂存)
        self.bridge_state = BridgeState.IDLE    # 本节点状态机
        self.last_goal_stamp = rospy.Time(0)    # 上次收到目标的时间戳
        self.last_setpoint_stamp = rospy.Time(0)
        self.takeoff_complete = False

        # ====== 订阅 ======
        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self._on_state, queue_size=10)
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10)
        self.goal_sub = rospy.Subscriber(
            "/uav/goal_pose", PoseStamped, self._on_goal, queue_size=10)
        self.cmd_sub = rospy.Subscriber(
            "/uav/state_cmd", String, self._on_cmd, queue_size=10)

        # ====== 发布 ======
        self.sp_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher(
            "/uav/bridge_status", String, queue_size=10, latch=True)

        # ====== 服务 ======
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # 发布初始状态
        self._publish_status("INITIALIZED")

        rospy.loginfo("[Bridge] Initialization complete. Waiting for FCU...")

    # --------------------------------------------------------------
    # 回调函数
    # --------------------------------------------------------------
    def _on_state(self, msg: State):
        self.state = msg

    def _on_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def _on_goal(self, msg: PoseStamped):
        """算法层发布新目标"""
        self.goal_pose = msg
        self.last_goal_stamp = rospy.Time.now()
        rospy.loginfo("[Bridge] New goal received: (%.2f, %.2f, %.2f)",
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _on_cmd(self, msg: String):
        """接收控制命令"""
        cmd = msg.data.strip().upper()
        rospy.loginfo("[Bridge] State command received: %s", cmd)

        if cmd == "TAKEOFF":
            self._start_takeoff()
        elif cmd == "LAND":
            self.bridge_state = BridgeState.LAND
        elif cmd == "RTL":
            self.bridge_state = BridgeState.RTL
        elif cmd == "HOVER":
            self.bridge_state = BridgeState.HOVER
        elif cmd == "GOTO":
            # 如果已有 pending_goal, 立即切换
            if self.pending_goal is not None:
                self.goal_pose = self.pending_goal
                self.last_goal_stamp = rospy.Time.now()
                self.bridge_state = BridgeState.FLYING
        elif cmd == "EMERGENCY":
            self.bridge_state = BridgeState.EMERGENCY
        else:
            rospy.logwarn("[Bridge] Unknown command: %s", cmd)

    # --------------------------------------------------------------
    # 核心逻辑
    # --------------------------------------------------------------
    def _publish_status(self, status: str):
        """发布桥接器状态"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _make_pose(self, x: float, y: float, z: float, yaw: float = 0.0) -> PoseStamped:
        """构造 PoseStamped"""
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        q = quaternion_from_euler(0, 0, yaw)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p

    def _publish_setpoint(self, pose: PoseStamped):
        """发布 setpoint（带时间戳更新）"""
        pose.header.stamp = rospy.Time.now()
        self.sp_pub.publish(pose)
        self.last_setpoint_stamp = rospy.Time.now()

    def _wait_for_fcu(self, timeout_s: float = 30.0):
        """等待飞控连接"""
        start = rospy.Time.now()
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.state.connected:
            if (rospy.Time.now() - start).to_sec() > timeout_s:
                raise RuntimeError("[Bridge] Timeout waiting for FCU connection")
            r.sleep()
        rospy.loginfo("[Bridge] FCU connected")

    def _arm(self) -> bool:
        """解锁"""
        try:
            resp = self.arming_srv(True)
            if resp.success:
                rospy.loginfo("[Bridge] Armed successfully")
            else:
                rospy.logwarn("[Bridge] Arm request rejected")
            return bool(resp.success)
        except rospy.ServiceException as e:
            rospy.logerr("[Bridge] Arm service call failed: %s", e)
            return False

    def _disarm(self) -> bool:
        """上锁"""
        try:
            resp = self.arming_srv(False)
            return bool(resp.success)
        except rospy.ServiceException:
            return False

    def _set_mode(self, mode: str) -> bool:
        """切换飞控模式"""
        try:
            resp = self.set_mode_srv(custom_mode=mode)
            if resp.mode_sent:
                rospy.loginfo("[Bridge] Mode set to %s", mode)
            else:
                rospy.logwarn("[Bridge] Mode %s rejected", mode)
            return bool(resp.mode_sent)
        except rospy.ServiceException as e:
            rospy.logerr("[Bridge] SetMode service call failed: %s", e)
            return False

    def _start_takeoff(self):
        """初始化起飞流程"""
        if self.bridge_state in (BridgeState.TAKEOFF, BridgeState.FLYING, BridgeState.LAND):
            rospy.logwarn("[Bridge] Already in %s, ignoring takeoff", self.bridge_state)
            return
        self.bridge_state = BridgeState.TAKEOFF

    # --------------------------------------------------------------
    # 主循环
    # --------------------------------------------------------------
    def run(self):
        """主运行循环"""
        rospy.loginfo("[Bridge] Waiting for FCU...")
        self._wait_for_fcu()
        rospy.loginfo("[Bridge] FCU ready. State: %s, Armed: %s",
                      self.state.connected, self.state.armed)

        rate = rospy.Rate(self.rate_hz)

        # 初始悬停点 (当前位置正上方 takeoff_alt)
        hover_pose = None

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # ====== 状态机 ======
            prev_state = self.bridge_state

            if self.bridge_state == BridgeState.IDLE:
                # 等待指令
                self._publish_status("IDLE")
                # 空转发布当前姿态 setpoint 以便后续切换 OFFBOARD
                sp = self._make_pose(
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z)
                self._publish_setpoint(sp)

            elif self.bridge_state == BridgeState.TAKEOFF:
                self._publish_status("TAKEOFF")
                # 构造起飞点 (保持当前 x,y, 上升到 takeoff_alt)
                x0 = self.current_pose.pose.position.x
                y0 = self.current_pose.pose.position.y
                hover_pose = self._make_pose(x0, y0, float(self.takeoff_alt))

                # 步骤1: 预发送 setpoint 2秒 (PX4 要求)
                rospy.loginfo("[Bridge] Pre-publishing setpoints for OFFBOARD...")
                for _ in range(int(2.0 * self.rate_hz)):
                    self._publish_setpoint(hover_pose)
                    rate.sleep()
                    if rospy.is_shutdown():
                        return

                # 步骤2: 解锁
                rospy.loginfo("[Bridge] Arming...")
                if not self._arm():
                    rospy.logerr("[Bridge] Failed to arm!")
                    self.bridge_state = BridgeState.IDLE
                    continue

                # 步骤3: 切换到 OFFBOARD
                rospy.loginfo("[Bridge] Setting OFFBOARD mode...")
                if not self._set_mode("OFFBOARD"):
                    rospy.logerr("[Bridge] Failed to set OFFBOARD!")
                    self._disarm()
                    self.bridge_state = BridgeState.IDLE
                    continue

                # 步骤4: 持续发布起飞 setpoint
                rospy.loginfo("[Bridge] Taking off to %.2fm...", self.takeoff_alt)
                # 等待到达目标高度 (或超时)
                takeoff_start = now
                altitude_ok = False
                while not rospy.is_shutdown():
                    self._publish_setpoint(hover_pose)
                    rate.sleep()
                    if rospy.is_shutdown():
                        return
                    curr_z = self.current_pose.pose.position.z
                    rospy.loginfo_throttle(1.0, "[Bridge] Current altitude: %.2f", curr_z)
                    if curr_z >= self.takeoff_alt * 0.95:
                        altitude_ok = True
                        break
                    if (rospy.Time.now() - takeoff_start).to_sec() > 15.0:
                        rospy.logwarn("[Bridge] Takeoff altitude timeout (%.2f < %.2f)",
                                      curr_z, self.takeoff_alt)
                        break

                if altitude_ok:
                    rospy.loginfo("[Bridge] Takeoff complete at %.2fm", curr_z)
                else:
                    rospy.logwarn("[Bridge] Takeoff may not have reached target altitude")

                # 进入飞行状态
                self.takeoff_complete = True
                self.bridge_state = BridgeState.FLYING

            elif self.bridge_state == BridgeState.FLYING:
                self._publish_status("FLYING")
                # 检查是否有目标
                if self.goal_pose is not None:
                    # 计算是否到达目标 (带 approach_dist 前停距离)
                    dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
                    dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
                    dz = self.goal_pose.pose.position.z - self.current_pose.pose.position.z
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                    # 构造带 approach_dist 偏移的目标点
                    if dist > self.approach_dist:
                        # 还没到目标：朝目标方向前进，但停在 approach_dist 处
                        ratio = 1.0 - self.approach_dist / dist if dist > 0 else 1.0
                        sp = self._make_pose(
                            self.current_pose.pose.position.x + dx * ratio,
                            self.current_pose.pose.position.y + dy * ratio,
                            self.current_pose.pose.position.z + dz * ratio)
                    else:
                        # 已进入 approach_dist 范围，悬停在当前位置
                        sp = self._make_pose(
                            self.current_pose.pose.position.x,
                            self.current_pose.pose.position.y,
                            self.current_pose.pose.position.z)
                        rospy.loginfo_throttle(2.0, "[Bridge] Approached goal (dist=%.2f <= %.2f)",
                                               dist, self.approach_dist)

                    self._publish_setpoint(sp)

                    # 检查目标超时 (如果算法层停止了发目标，则超时悬停)
                    dt = (now - self.last_goal_stamp).to_sec()
                    if dt > self.goal_timeout:
                        rospy.logwarn("[Bridge] Goal timeout (%.1fs > %.1fs), hovering",
                                      dt, self.goal_timeout)
                        self.bridge_state = BridgeState.HOVER
                else:
                    # 没有目标，悬停
                    sp = self._make_pose(
                        self.current_pose.pose.position.x,
                        self.current_pose.pose.position.y,
                        self.current_pose.pose.position.z)
                    self._publish_setpoint(sp)

            elif self.bridge_state == BridgeState.HOVER:
                self._publish_status("HOVER")
                sp = self._make_pose(
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z)
                self._publish_setpoint(sp)

            elif self.bridge_state == BridgeState.LAND:
                self._publish_status("LAND")
                rospy.loginfo("[Bridge] Switching to AUTO.LAND...")
                if self._set_mode("AUTO.LAND"):
                    # 发布几秒 setpoint 防止掉落
                    for _ in range(int(2.0 * self.rate_hz)):
                        sp = self._make_pose(
                            self.current_pose.pose.position.x,
                            self.current_pose.pose.position.y,
                            self.current_pose.pose.position.z)
                        self._publish_setpoint(sp)
                        rate.sleep()
                    self.bridge_state = BridgeState.COMPLETE
                else:
                    # 降级到 AUTO.RTL
                    rospy.logwarn("[Bridge] AUTO.LAND failed, trying AUTO.RTL")
                    if self._set_mode("AUTO.RTL"):
                        self.bridge_state = BridgeState.COMPLETE
                    else:
                        rospy.logerr("[Bridge] All landing modes failed!")
                        self.bridge_state = BridgeState.EMERGENCY

            elif self.bridge_state == BridgeState.RTL:
                self._publish_status("RTL")
                rospy.loginfo("[Bridge] Switching to AUTO.RTL...")
                if self._set_mode("AUTO.RTL"):
                    self.bridge_state = BridgeState.COMPLETE
                else:
                    rospy.logerr("[Bridge] AUTO.RTL failed!")
                    self.bridge_state = BridgeState.EMERGENCY

            elif self.bridge_state == BridgeState.EMERGENCY:
                self._publish_status("EMERGENCY")
                rospy.logerr("[Bridge] EMERGENCY - disarming!")
                self._disarm()
                self.bridge_state = BridgeState.IDLE

            elif self.bridge_state == BridgeState.COMPLETE:
                self._publish_status("COMPLETE")
                rospy.loginfo("[Bridge] Mission complete. Waiting for new commands.")
                # 完成后续传保持当前位置 setpoint
                sp = self._make_pose(
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z)
                self._publish_setpoint(sp)
                # 重置状态为 IDLE (允许再次起飞)
                self.bridge_state = BridgeState.IDLE

            # 状态变化时打印日志
            if prev_state != self.bridge_state:
                rospy.loginfo("[Bridge] State: %s -> %s", prev_state, self.bridge_state)

            rate.sleep()


def main():
    rospy.init_node("setpoint_bridge", anonymous=False)
    node = SetpointBridge()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[Bridge] Unhandled exception: %s", e)
        raise


if __name__ == "__main__":
    main()
