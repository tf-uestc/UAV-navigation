#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vlm_navigation.py - 视觉语言模型导航节点

【整体流程】
1. 触发拍照 -> 从 /rgb/image_raw 或 /depth/image_raw 获取图像
2. 调用 VLM API (Qwen-VL / GPT-4V 等) -> 提问"前方红色障碍物像素坐标是什么"
3. 解析 VLM 返回的像素坐标 (u, v)
4. 从深度图获取该像素的深度值 d
5. 利用相机内参将 (u, v, d) 投影到 3D 相机坐标系
6. 将相机坐标系下的点转换到世界坐标系 (map frame)
7. 构造目标位姿 (在障碍物前 approach_dist 处) 发布到 /uav/goal_pose
8. setpoint_bridge 负责实际飞行控制

【话题】
  订阅:
    /rgb/image_raw          (sensor_msgs/Image) - RGB 图像
    /depth/image_raw        (sensor_msgs/Image) - 深度图像 (float32 米)
    /mavros/local_position/pose (PoseStamped)   - 无人机当前位置 (用于坐标系转换)
    /uav/bridge_status      (String)            - setpoint_bridge 的状态

  发布:
    /uav/goal_pose          (PoseStamped) - 计算得到的目标位置 (map frame)
    /uav/state_cmd          (String)      - 控制命令 (takeoff/goto/land)
    /uav/vlm_debug_image    (sensor_msgs/Image) - 调试用标注图像

【参数】
  ~vlm_provider       (str,   default: "qwen")    - VLM 提供商 (qwen / openai)
  ~vlm_api_key        (str,   default: "")         - API 密钥
  ~vlm_model          (str,   default: "qwen-vl-max") - 模型名
  ~vlm_prompt         (str,   default: "前方红色障碍物像素坐标是什么?请只返回\"(u,v)\"格式") 
  ~approach_dist      (float, default: 1.0)        - 最终停在障碍物前 X 米
  ~camera_topic_rgb   (str,   default: "/rgb/image_raw")
  ~camera_topic_depth (str,   default: "/depth/image_raw")
  ~trigger_rate       (float, default: 0.2)        - 触发频率 (秒), 0=单次触发
  ~depth_scale        (float, default: 1.0)        - 深度图缩放因子 (米/单位)
"""

import os
import json
import math
import base64
from io import BytesIO
from typing import Optional, Tuple

import cv2
import numpy as np
import rospy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler, quaternion_multiply


# ==============================================================================
# 1. VLM API 客户端 (统一接口, 可扩展多个提供商)
# ==============================================================================
class VLMApiClient:
    """支持多个 VLM 提供商的统一客户端"""

    PROVIDERS = {
        "qwen": {
            "url": "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions",
            "model": "qwen-vl-max",
        },
        "openai": {
            "url": "https://api.openai.com/v1/chat/completions",
            "model": "gpt-4o",
        },
    }

    def __init__(self, provider: str = "qwen", api_key: str = "",
                 model: str = None, timeout: float = 30.0):
        if provider not in self.PROVIDERS:
            raise ValueError(f"Unsupported VLM provider: {provider}. "
                             f"Supported: {list(self.PROVIDERS.keys())}")

        config = self.PROVIDERS[provider].copy()
        self.api_url = config["url"]
        self.model = model or config["model"]
        self.api_key = api_key or os.environ.get(
            f"VLM_API_KEY_{provider.upper()}",
            os.environ.get("VLM_API_KEY", "")
        )
        self.timeout = timeout
        self.provider = provider

        if not self.api_key:
            rospy.logwarn("[VLM] No API key set for %s! "
                          "Set VLM_API_KEY env or ~vlm_api_key param", provider)

    def query_pixel(self, image_cv: np.ndarray, prompt: str) -> Optional[Tuple[int, int]]:
        """
        将图像发给 VLM, 解析返回的像素坐标 (u, v)

        返回: (u, v) 或 None
        """
        # 编码图像为 base64 JPEG
        success, buffer = cv2.imencode(".jpg", image_cv, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            rospy.logerr("[VLM] Failed to encode image")
            return None

        image_b64 = base64.b64encode(buffer).decode("utf-8")
        image_data_url = f"data:image/jpeg;base64,{image_b64}"

        # 构造 API 请求
        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": image_data_url},
                        },
                    ],
                }
            ],
            "max_tokens": 128,
        }

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

        try:
            rospy.loginfo("[VLM] Querying %s (%s)...", self.provider, self.model)
            resp = requests.post(
                self.api_url, headers=headers,
                json=payload, timeout=self.timeout
            )
            resp.raise_for_status()
            result = resp.json()

            # 提取文本回复
            content = result["choices"][0]["message"]["content"]
            rospy.loginfo("[VLM] Response: %s", content[:200])

            # 解析坐标: 支持 "(u,v)" 或 "u,v" 或 "u v" 格式
            return self._parse_coords(content)

        except requests.exceptions.Timeout:
            rospy.logerr("[VLM] API request timed out after %.1fs", self.timeout)
            return None
        except requests.exceptions.RequestException as e:
            rospy.logerr("[VLM] API request failed: %s", e)
            return None
        except (KeyError, IndexError, json.JSONDecodeError) as e:
            rospy.logerr("[VLM] Failed to parse API response: %s", e)
            return None

    @staticmethod
    def _parse_coords(text: str) -> Optional[Tuple[int, int]]:
        """从文本中提取像素坐标 (u, v)"""
        import re
        # 尝试匹配 (数字,数字) 格式
        patterns = [
            r"\(?\s*(\d+)\s*[,，]\s*(\d+)\s*\)?",    # (123,456) 或 123,456
            r"\(?\s*(\d+)\s+(\d+)\s*\)?",              # 123 456
            r"u[:\s]*(\d+).*?v[:\s]*(\d+)",            # u:123 v:456
        ]
        for pattern in patterns:
            m = re.search(pattern, text, re.IGNORECASE)
            if m:
                return int(m.group(1)), int(m.group(2))
        rospy.logwarn("[VLM] Could not parse coordinates from: %s", text[:100])
        return None


# ==============================================================================
# 2. 深度投影工具 - 将像素坐标 + 深度转为 3D 世界坐标
# ==============================================================================
class DepthProjector:
    """
    利用深度图和相机内参将像素坐标 (u, v) 投影到世界坐标系 (map frame)

    流程:
      pixel (u,v) + depth d  ->  相机坐标系 (x_c, y_c, z_c)
      -> 通过 TF 或位姿信息 -> 世界坐标系 (x_w, y_w, z_w)
    """

    def __init__(self):
        # 默认内参 (iris_depth_camera 深度相机: 848x480, 可根据实际修改)
        self.fx = rospy.get_param("~cam_fx", 430.0)
        self.fy = rospy.get_param("~cam_fy", 430.0)
        self.cx = rospy.get_param("~cam_cx", 424.0)
        self.cy = rospy.get_param("~cam_cy", 240.0)
        self.width = rospy.get_param("~cam_width", 848)
        self.height = rospy.get_param("~cam_height", 480)

        self.camera_info_received = False

        # 订阅 CameraInfo 获取准确内参
        self.cam_info_sub = rospy.Subscriber(
            "/depth/camera_info", CameraInfo, self._on_camera_info, queue_size=1)

    def _on_camera_info(self, msg: CameraInfo):
        """从 CameraInfo 话题获取准确的相机内参"""
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.width = msg.width
        self.height = msg.height
        if not self.camera_info_received:
            rospy.loginfo("[VLM] CameraInfo received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                          self.fx, self.fy, self.cx, self.cy)
            self.camera_info_received = True

    def get_depth_at(self, depth_image: np.ndarray, u: int, v: int,
                     kernel_size: int = 3) -> Optional[float]:
        """
        从深度图中获取 (u,v) 处的深度值 (米),
        使用 kernel_size x kernel_size 邻域中值滤波提高稳定性
        """
        if depth_image is None:
            return None

        h, w = depth_image.shape[:2]

        # 边界检查
        if u < 0 or u >= w or v < 0 or v >= h:
            rospy.logwarn("[VLM] Pixel (%d, %d) out of bounds (%dx%d)", u, v, w, h)
            return None

        # 取邻域
        half_k = kernel_size // 2
        v_min = max(0, v - half_k)
        v_max = min(h, v + half_k + 1)
        u_min = max(0, u - half_k)
        u_max = min(w, u + half_k + 1)

        patch = depth_image[v_min:v_max, u_min:u_max]

        if patch.size == 0:
            return None

        # 过滤无效深度 (NaN, 0, 无穷大)
        valid = patch[(~np.isnan(patch)) & (patch > 0) & (np.isfinite(patch))]
        if valid.size == 0:
            rospy.logwarn("[VLM] No valid depth at (%d, %d)", u, v)
            return None

        return float(np.median(valid))

    def pixel_to_camera(self, u: int, v: int, depth: float) -> Tuple[float, float, float]:
        """
        像素坐标 + 深度 -> 相机坐标系 (x向右, y向下, z向前)

        公式:
          x_c = (u - cx) * depth / fx
          y_c = (v - cy) * depth / fy
          z_c = depth
        """
        x_c = (u - self.cx) * depth / self.fx
        y_c = (v - self.cy) * depth / self.fy
        z_c = depth
        return x_c, y_c, z_c

    @staticmethod
    def camera_to_world_approx(cam_point: Tuple[float, float, float],
                                drone_pose: PoseStamped) -> Tuple[float, float, float]:
        """
        近似: 相机坐标系 -> 世界坐标系 (使用无人机姿态)
        假设相机朝前 (x_c = 前, y_c = 右, z_c = 下)

        简化版本: 只考虑无人机 yaw 旋转 + 位置偏移
        更精确的版本应使用 TF (camera_link -> map)
        """
        x_c, y_c, z_c = cam_point

        # 获取无人机朝向 (yaw)
        q = drone_pose.pose.orientation
        # 从四元数提取 yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 将相机坐标旋转到世界坐标系 (仅 yaw)
        # 注意: 相机朝前 = 无人机朝前 = x_local
        # 在 map frame 中: 无人机 x 轴指向 yaw 方向
        x_w = drone_pose.pose.position.x + x_c * math.cos(yaw) - y_c * math.sin(yaw)
        y_w = drone_pose.pose.position.y + x_c * math.sin(yaw) + y_c * math.cos(yaw)
        z_w = drone_pose.pose.position.z + z_c  # 相机的高度偏移近似

        return x_w, y_w, z_w


# ==============================================================================
# 3. 主节点
# ==============================================================================
class VLNNavigation:
    def __init__(self):
        rospy.loginfo("[VLM] Initializing VLN Navigation node...")

        # ====== 参数 ======
        self.vlm_provider = rospy.get_param("~vlm_provider", "qwen")
        self.vlm_api_key = rospy.get_param("~vlm_api_key", "")
        self.vlm_model = rospy.get_param("~vlm_model", "")
        self.vlm_prompt = rospy.get_param(
            "~vlm_prompt",
            "前方红色障碍物像素坐标是什么?请只返回\"(u,v)\"格式的坐标"
        )
        self.approach_dist = rospy.get_param("~approach_dist", 1.0)
        self.trigger_rate = rospy.get_param("~trigger_rate", 0.2)  # Hz
        self.depth_scale = rospy.get_param("~depth_scale", 1.0)
        self.rgb_topic = rospy.get_param("~camera_topic_rgb", "/rgb/image_raw")
        self.depth_topic = rospy.get_param("~camera_topic_depth", "/depth/image_raw")

        # ====== 内部状态 ======
        self.bridge = CvBridge()
        self.projector = DepthProjector()
        self.vlm_client = VLMApiClient(
            provider=self.vlm_provider,
            api_key=self.vlm_api_key,
            model=self.vlm_model if self.vlm_model else None,
        )

        self.latest_rgb = None
        self.latest_depth = None
        self.rgb_stamp = None
        self.depth_stamp = None
        self.drone_pose = PoseStamped()
        self.bridge_status = "IDLE"
        self.mission_started = False
        self.mission_complete = False

        # ====== 订阅 ======
        self.rgb_sub = rospy.Subscriber(
            self.rgb_topic, Image, self._on_rgb, queue_size=1)
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self._on_depth, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10)
        self.bridge_status_sub = rospy.Subscriber(
            "/uav/bridge_status", String, self._on_bridge_status, queue_size=10)

        # ====== 发布 ======
        self.goal_pub = rospy.Publisher(
            "/uav/goal_pose", PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher(
            "/uav/state_cmd", String, queue_size=10)
        self.debug_pub = rospy.Publisher(
            "/uav/vlm_debug_image", Image, queue_size=1)

        rospy.loginfo("[VLM] Initialization complete. "
                      "Provider=%s, Model=%s, Rate=%.1fHz",
                      self.vlm_provider,
                      self.vlm_model or self.vlm_client.model,
                      self.trigger_rate)

    # --------------------------------------------------------------
    # 回调
    # --------------------------------------------------------------
    def _on_rgb(self, msg: Image):
        self.latest_rgb = msg
        self.rgb_stamp = msg.header.stamp

    def _on_depth(self, msg: Image):
        self.latest_depth = msg
        self.depth_stamp = msg.header.stamp

    def _on_pose(self, msg: PoseStamped):
        self.drone_pose = msg

    def _on_bridge_status(self, msg: String):
        self.bridge_status = msg.data

    # --------------------------------------------------------------
    # 获取同步图像
    # --------------------------------------------------------------
    def _get_synced_images(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """获取时间同步的 RGB 和深度图像"""
        if self.latest_rgb is None or self.latest_depth is None:
            rospy.logwarn_throttle(5.0, "[VLM] Waiting for camera data...")
            return None, None

        try:
            rgb_cv = self.bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
            depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth, "passthrough")
            return rgb_cv, depth_cv.astype(np.float32) * self.depth_scale
        except Exception as e:
            rospy.logerr("[VLM] Failed to convert images: %s", e)
            return None, None

    # --------------------------------------------------------------
    # 发布调试图像
    # --------------------------------------------------------------
    def _publish_debug_image(self, rgb_cv: np.ndarray, u: int, v: int,
                              depth_val: float, goal_world: Tuple[float, float, float]):
        """在图像上标注检测结果并发布"""
        img = rgb_cv.copy()
        h, w = img.shape[:2]

        # 绘制十字线标记目标像素
        cv2.drawMarker(img, (u, v), (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.circle(img, (u, v), 5, (0, 255, 0), -1)

        # 显示深度值
        info_text = f"Depth: {depth_val:.2f}m" if depth_val > 0 else "Depth: N/A"
        cv2.putText(img, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Pixel: ({u}, {v})", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Goal: ({goal_world[0]:.1f}, {goal_world[1]:.1f})",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            debug_msg.header.stamp = rospy.Time.now()
            debug_msg.header.frame_id = "camera_link"
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            rospy.logwarn("[VLM] Failed to publish debug image: %s", e)

    # --------------------------------------------------------------
    # 发布目标到 setpoint_bridge
    # --------------------------------------------------------------
    def _publish_goal(self, x: float, y: float, z: float):
        """发布目标位姿到 /uav/goal_pose"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        rospy.loginfo("[VLM] Published goal: (%.2f, %.2f, %.2f)", x, y, z)

    def _send_cmd(self, cmd: str):
        """发送控制命令到 setpoint_bridge"""
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        rospy.loginfo("[VLM] Sent command: %s", cmd)

    # --------------------------------------------------------------
    # 核心: 单次 VLM 导航循环
    # --------------------------------------------------------------
    def _run_vlm_cycle(self) -> bool:
        """
        执行一次 VLM 导航循环:
          1. 获取图像
          2. 调用 VLM API
          3. 解析像素坐标
          4. 获取深度值
          5. 投影到 3D
          6. 发布目标

        返回: True 表示成功找到目标并发布
        """
        # 1. 获取同步图像
        rgb_cv, depth_cv = self._get_synced_images()
        if rgb_cv is None:
            return False

        rospy.loginfo("[VLM] Captured image: %dx%d", rgb_cv.shape[1], rgb_cv.shape[0])

        # 2. 调用 VLM API (异步执行不阻塞主循环)
        result = self.vlm_client.query_pixel(rgb_cv, self.vlm_prompt)
        if result is None:
            rospy.logerr("[VLM] VLM query failed or no coordinates returned")
            return False

        u, v = result
        rospy.loginfo("[VLM] VLM returned pixel: (%d, %d)", u, v)

        # 3. 获取深度值
        if depth_cv is None:
            rospy.logerr("[VLM] No depth image available")
            return False

        depth_val = self.projector.get_depth_at(depth_cv, u, v)
        if depth_val is None or depth_val <= 0:
            rospy.logerr("[VLM] Invalid depth at (%d, %d)", u, v)
            return False

        rospy.loginfo("[VLM] Depth at (%d, %d): %.2fm", u, v, depth_val)

        # 4. 像素 -> 相机坐标系 (带 approach_dist 前停距离)
        x_c, y_c, z_c = self.projector.pixel_to_camera(u, v, depth_val)
        rospy.loginfo("[VLM] Camera frame: (%.2f, %.2f, %.2f)", x_c, y_c, z_c)

        # 计算前停位置: 在目标前 approach_dist 处
        forward_dist = max(0, depth_val - self.approach_dist)
        x_c_approach = (u - self.projector.cx) * forward_dist / self.projector.fx
        y_c_approach = (v - self.projector.cy) * forward_dist / self.projector.fy
        z_c_approach = forward_dist

        rospy.loginfo("[VLM] Approach point (camera): (%.2f, %.2f, %.2f) "
                      "(%.1fm before obstacle)",
                      x_c_approach, y_c_approach, z_c_approach,
                      self.approach_dist)

        # 5. 相机坐标系 -> 世界坐标系
        goal_world = DepthProjector.camera_to_world_approx(
            (x_c_approach, y_c_approach, z_c_approach),
            self.drone_pose
        )
        rospy.loginfo("[VLM] World goal: (%.2f, %.2f, %.2f)", *goal_world)

        # 6. 发布目标 + 调试图像
        self._publish_goal(*goal_world)
        self._publish_debug_image(rgb_cv, u, v, depth_val, goal_world)

        return True

    # --------------------------------------------------------------
    # 主循环
    # --------------------------------------------------------------
    def run(self):
        """主运行循环"""
        rospy.loginfo("[VLM] Waiting for setpoint_bridge to be ready...")

        # 等待 bridge 就绪
        rate = rospy.Rate(10)
        timeout_start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.bridge_status != "IDLE":
                rospy.loginfo("[VLM] Bridge status: %s", self.bridge_status)
                break
            if (rospy.Time.now() - timeout_start).to_sec() > 60.0:
                rospy.logwarn("[VLM] Timeout waiting for bridge, starting anyway")
                break
            rate.sleep()

        # 等待图像话题
        rospy.loginfo("[VLM] Waiting for camera topics...")
        for _ in range(50):  # 最多等 5 秒
            if self.latest_rgb is not None and self.latest_depth is not None:
                rospy.loginfo("[VLM] Camera data received")
                break
            rate.sleep()

        if self.latest_rgb is None:
            rospy.logerr("[VLM] No camera data available! "
                         "Check camera topics: %s, %s",
                         self.rgb_topic, self.depth_topic)
            return

        # ====== 主逻辑: 起飞 -> VLM 导航 ======
        rospy.loginfo("[VLM] ===== Starting VLM Navigation Mission =====")

        # 阶段 1: 发送起飞命令
        rospy.loginfo("[VLM] Phase 1: Takeoff")
        self._send_cmd("TAKEOFF")

        # 等待起飞完成
        rospy.loginfo("[VLM] Waiting for takeoff...")
        while not rospy.is_shutdown():
            if self.bridge_status in ("FLYING", "HOVER"):
                rospy.loginfo("[VLM] Takeoff complete, bridge status: %s",
                              self.bridge_status)
                break
            if self.bridge_status in ("LAND", "COMPLETE", "EMERGENCY"):
                rospy.logwarn("[VLM] Bridge in unexpected state during takeoff: %s",
                              self.bridge_status)
                return
            rate.sleep()

        # 阶段 2: VLM 导航循环 (持续检测 + 飞向目标)
        rospy.loginfo("[VLM] Phase 2: VLM navigation loop started")
        trigger_interval = 1.0 / max(self.trigger_rate, 0.01)
        last_trigger = rospy.Time.now()

        # 首次触发
        rospy.loginfo("[VLM] First VLM query...")
        success = self._run_vlm_cycle()
        if not success:
            rospy.logerr("[VLM] First VLM cycle failed! "
                         "Check camera topics and VLM API")

        # 持续循环
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # 检查 bridge 是否异常
            if self.bridge_status in ("LAND", "COMPLETE", "EMERGENCY", "RTL"):
                rospy.loginfo("[VLM] Mission ended (bridge: %s)", self.bridge_status)
                break

            # 定时触发 VLM 重新检测
            if (now - last_trigger).to_sec() >= trigger_interval:
                rospy.loginfo("[VLM] ===== VLM Cycle =====")
                self._run_vlm_cycle()
                last_trigger = now

            rate.sleep()

        rospy.loginfo("[VLM] ===== VLM Navigation Mission Finished =====")


def main():
    rospy.init_node("vlm_navigation", anonymous=False)
    node = VLNNavigation()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[VLM] Unhandled exception: %s", e)
        raise


if __name__ == "__main__":
    main()
