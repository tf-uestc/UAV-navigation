"""
qwen_vl_grounding.py — Qwen VL grounding detector via DashScope API.

Implements the Detector ABC from base_detector using Qwen-VL's
visual grounding capability (bbox output) with depth fusion.
"""
import base64
import json
import re
from typing import Optional, Tuple

import cv2
import numpy as np
import requests
import rospy
from sensor_msgs.msg import CameraInfo

from base_detector import Detection, Detector

# ---------------------------------------------------------------------------
# Prompt template and bbox parsing patterns
# ---------------------------------------------------------------------------
DEFAULT_PROMPT = "请框出图中的{target}。只输出一个框,不要解释。"

PATTERNS = [
    # Qwen2-VL format: <|box_start|>(x1,y1),(x2,y2)<|box_end|>
    r"<\|box_start\|>\((\d+),(\d+)\),\((\d+),(\d+)\)<\|box_end\|>",
    # Qwen-VL classic: <box>(x1,y1),(x2,y2)</box>
    r"<box>\s*\((\d+),(\d+)\)\s*,\s*\((\d+),(\d+)\)\s*</box>",
    # Fallback: bracketed array [x1,y1,x2,y2] or bare numbers
    r"\[?\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]?",
]

API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"


# ---------------------------------------------------------------------------
# QwenVLGroundingDetector
# ---------------------------------------------------------------------------
class QwenVLGroundingDetector(Detector):
    """Production Qwen-VL grounding detector with bbox parsing and depth fusion.

    Calls the DashScope OpenAI-compatible API with a grounding prompt,
    parses the returned bounding box, fuses with depth data, and produces
    a Detection with 3D camera-optical coordinates.
    """

    def __init__(self, api_key: str, model: str = "qwen-vl-max",
                 timeout: float = 30.0) -> None:
        self.api_key = api_key
        self.model = model
        self.timeout = timeout

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
    def detect(self,
               rgb: np.ndarray,
               depth: np.ndarray,
               target_text: str,
               cam_info: CameraInfo) -> Optional[Detection]:
        """Run grounding pipeline on a single frame.

        Returns a Detection on success, or None if any stage fails.
        """
        # 1. Call VLM API
        raw_text = self._call_api(rgb, target_text)
        if raw_text is None:
            return None

        # 2. Parse bbox from response
        h, w = rgb.shape[:2]
        bbox = self._parse_bbox(raw_text, w, h)
        if bbox is None:
            rospy.logwarn("Failed to parse bbox from: %s", raw_text[:100])
            return None

        x1, y1, x2, y2 = bbox

        # 3. Fuse with depth
        depth_m = self._depth_in_bbox(depth, x1, y1, x2, y2)
        if depth_m is None:
            return None

        # 4. Back-project center pixel to camera optical frame
        center_u = (x1 + x2) // 2
        center_v = (y1 + y2) // 2
        point_camera = self._pixel_to_camera_optical(
            center_u, center_v, depth_m, cam_info.K
        )

        # 5. Compute confidence from valid-depth ratio inside bbox
        bbox_w, bbox_h = x2 - x1, y2 - y1
        patch = depth[y1:y2, x1:x2]
        valid = patch[(~np.isnan(patch)) & (patch > 0) & np.isfinite(patch)]
        confidence = valid.size / max(bbox_w * bbox_h, 1)

        return Detection(
            bbox=bbox,
            point_camera=point_camera,
            depth_m=depth_m,
            confidence=confidence,
            raw={"raw_text": raw_text},
            target_text=target_text,
        )

    def healthcheck(self) -> bool:
        """No-API-ping healthcheck — always returns True."""
        return True

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _call_api(self, rgb: np.ndarray, target_text: str) -> Optional[str]:
        """Send grounding request to DashScope and return raw text response.

        Returns None on any error (timeout, network, parse).
        """
        prompt = DEFAULT_PROMPT.format(target=target_text)

        # Encode image as base64 JPEG (cv2.imencode expects BGR)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        success, buffer = cv2.imencode(".jpg", bgr,
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            rospy.logerr("Failed to encode image to JPEG")
            return None

        image_b64 = base64.b64encode(buffer).decode("utf-8")
        image_data_url = f"data:image/jpeg;base64,{image_b64}"

        payload = {
            "model": self.model,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": image_data_url}},
                ],
            }],
            "max_tokens": 128,
        }
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

        try:
            rospy.loginfo("Querying DashScope (%s)...", self.model)
            resp = requests.post(
                API_URL, headers=headers, json=payload,
                timeout=self.timeout,
            )
            resp.raise_for_status()
            result = resp.json()
            content = result["choices"][0]["message"]["content"]
            rospy.loginfo("VLM response (%d chars)", len(content))
            return content

        except requests.exceptions.Timeout:
            rospy.logerr("API request timed out after %.1fs", self.timeout)
            return None
        except requests.exceptions.RequestException as e:
            rospy.logerr("API request failed: %s", e)
            return None
        except (KeyError, IndexError, json.JSONDecodeError) as e:
            rospy.logerr("Failed to parse API response: %s", e)
            return None

    @staticmethod
    def _parse_bbox(text: str, img_w: int,
                    img_h: int) -> Optional[Tuple[int, int, int, int]]:
        """Extract pixel-coordinate bbox from VLM text response.

        Tries 3 regex patterns in order (Qwen2-VL, classic, fallback),
        then applies the normalized→pixel heuristic when coordinates
        appear to be in the [0,1000] convention.
        """
        for pattern in PATTERNS:
            m = re.search(pattern, text)
            if m:
                x1, y1, x2, y2 = (int(m.group(i)) for i in range(1, 5))
                break
        else:
            return None

        # Normalization heuristic:
        # Qwen may output coords normalized to [0,1000] or raw pixels.
        bbox_w = x2 - x1
        bbox_h = y2 - y1
        bbox_area_ratio = (bbox_w * bbox_h) / (img_w * img_h) if (img_w * img_h) > 0 else 0.0

        if max(x1, y1, x2, y2) <= 1000 and (img_w > 1000 or img_h > 1000
                                             or bbox_area_ratio < 0.5):
            # Normalized → scale to pixel coordinates
            x1 = int(x1 * img_w / 1000)
            y1 = int(y1 * img_h / 1000)
            x2 = int(x2 * img_w / 1000)
            y2 = int(y2 * img_h / 1000)

        # Clamp to image bounds
        x1 = max(0, min(x1, img_w - 1))
        y1 = max(0, min(y1, img_h - 1))
        x2 = max(0, min(x2, img_w - 1))
        y2 = max(0, min(y2, img_h - 1))

        # Ensure non-degenerate
        if x2 <= x1 or y2 <= y1:
            return None

        return (x1, y1, x2, y2)

    @staticmethod
    def _depth_in_bbox(depth: np.ndarray, x1: int, y1: int,
                       x2: int, y2: int) -> Optional[float]:
        """Compute median depth inside bbox region.

        Requires ≥30% valid depth pixels.  Returns None otherwise.
        """
        h, w = depth.shape
        x1 = max(0, min(x1, w - 1))
        x2 = max(x1 + 1, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(y1 + 1, min(y2, h))

        patch = depth[y1:y2, x1:x2]
        valid = patch[(~np.isnan(patch)) & (patch > 0) & np.isfinite(patch)]

        if valid.size < 0.3 * patch.size:
            return None

        return float(np.median(valid))

    @staticmethod
    def _pixel_to_camera_optical(u: float, v: float, depth_m: float,
                                 K: np.ndarray) -> Tuple[float, float, float]:
        """Back-project a pixel to 3D in camera optical frame.

        K is row-major 3x3: [fx, 0, cx, 0, fy, cy, 0, 0, 1].
        Camera optical frame: z-forward, x-right, y-down (ROS convention).
        """
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return (x, y, z)
