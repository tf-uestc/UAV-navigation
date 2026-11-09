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

from base_detector import Detection, DetectionRejected, Detector
from navigation_core.perception import parse_bbox, validate_intrinsics

# ---------------------------------------------------------------------------
# Prompt template and bbox parsing patterns
# ---------------------------------------------------------------------------
DEFAULT_PROMPT = "查找目标：{target}。图像宽{width}高{height}。只输出JSON，不要解释。"

PATTERNS = [
    # Qwen2-VL format: <|box_start|>(x1,y1),(x2,y2)<|box_end|>
    r"<\|box_start\|>\((\d+),(\d+)\),\((\d+),(\d+)\)<\|box_end\|>",
    # Qwen-VL classic: <box>(x1,y1),(x2,y2)</box>
    r"<box>\s*\((\d+),(\d+)\)\s*,\s*\((\d+),(\d+)\)\s*</box>",
    # Fallback: bracketed array [x1,y1,x2,y2] or bare numbers
    r"\[?\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]?",
]

OMNI_MODEL = "qwen3.8-omni-flash"
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

    def __init__(self, api_key: str, model: str = OMNI_MODEL,
                 timeout: float = 90.0, coordinate_system: Optional[str] = None,
                 allow_legacy_bbox: bool = False) -> None:
        self.api_key = api_key
        self.model = model
        self.timeout = timeout
        if coordinate_system is None:
            coordinate_system = 'normalized_1000' if model == OMNI_MODEL else 'pixel'
        if coordinate_system not in ('pixel', 'normalized_1000'):
            raise ValueError('unsupported bbox coordinate system')
        self.coordinate_system = coordinate_system
        self.allow_legacy_bbox = allow_legacy_bbox

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
    def detect(self,
               rgb: np.ndarray,
               depth: np.ndarray,
               target_text: str,
               cam_info: CameraInfo) -> Optional[Detection]:
        """Run grounding pipeline on a single frame.

        Returns a Detection on success and classifies rejected observations.
        """
        validate_intrinsics(cam_info.K)
        if rgb.shape[:2] != depth.shape[:2]:
            raise ValueError('RGB/depth must be aligned')
        # 1. Call VLM API
        raw_text = self._call_api(rgb, target_text)
        if raw_text is None:
            raise DetectionRejected("invalid_api_response")

        # 2. Parse bbox from response
        h, w = rgb.shape[:2]
        bbox = self._parse_bbox(raw_text, w, h, self.coordinate_system, self.allow_legacy_bbox)
        if bbox is None:
            try:
                payload = json.loads(raw_text)
            except (ValueError, TypeError):
                payload = None
            reason = ("target_not_found" if isinstance(payload, dict) and payload.get("found") is False
                      else "invalid_bbox_response")
            raise DetectionRejected(reason)

        x1, y1, x2, y2 = bbox
        u, v, depth_m, support = self._foreground_depth(depth[y1:y2, x1:x2])
        point_camera = self._pixel_to_camera_optical(x1+u, y1+v, depth_m, cam_info.K)
        confidence = support / ((x2-x1)*(y2-y1))

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

        Raises DetectionRejected with a stable reason if the request fails.
        """
        prompt = DEFAULT_PROMPT.format(target=target_text, width=rgb.shape[1], height=rgb.shape[0])
        units = ("以图像宽高分别将x和y线性归一化到0至1000"
                 if self.coordinate_system == "normalized_1000" else "使用原图像素坐标")
        prompt += f"坐标协议为{self.coordinate_system}（{units}），bbox顺序为[x1,y1,x2,y2]，仅一个目标。"
        prompt += "找到时格式：" + json.dumps(
            {"found": True, "bbox": [0, 0, 10, 10], "coordinate_system": self.coordinate_system})
        prompt += '；未找到时只输出{"found":false}。禁止猜测不可见目标。'

        # Encode image as base64 JPEG (cv2.imencode expects BGR)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        success, buffer = cv2.imencode(".jpg", bgr,
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            raise DetectionRejected("image_encode_failed")

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
        if self.model == OMNI_MODEL:
            payload.update(modalities=["text"], reasoning_effort="none",
                           response_format={"type": "json_object"}, stream=False)
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
            if not isinstance(content, str) or not content.strip():
                raise DetectionRejected("invalid_api_response")
            rospy.loginfo("VLM response (%d chars)", len(content))
            return content

        except requests.exceptions.Timeout:
            raise DetectionRejected("api_timeout") from None
        except requests.exceptions.RequestException as exc:
            status = getattr(getattr(exc, "response", None), "status_code", None)
            raise DetectionRejected(f"api_http_{status}" if status else "api_network_error") from None
        except (KeyError, IndexError, TypeError, json.JSONDecodeError):
            raise DetectionRejected("invalid_api_response") from None

    @staticmethod
    def _parse_bbox(text, img_w, img_h, coordinate_system="pixel", allow_legacy=False):
        """Never infer coordinate units from their numerical range."""
        return parse_bbox(text, img_w, img_h, coordinate_system, allow_legacy)

    @staticmethod
    def _foreground_depth(patch: np.ndarray):
        """Pick the closest depth band with enough support and project its pixels."""
        positive = np.isfinite(patch)
        positive[positive] = patch[positive] > 0
        v, u = np.nonzero(positive)
        minimum = max(3, int(np.ceil(.10*patch.size)))
        if len(u) < minimum:
            raise DetectionRejected("invalid_target_depth")
        depths = patch[v, u]
        order = np.argsort(depths)
        ordered = depths[order]
        splits = np.flatnonzero(np.diff(ordered) > np.maximum(.15, .05*ordered[:-1]))+1
        for group in np.split(order, splits):
            if len(group) >= minimum:
                return float(np.median(u[group])), float(np.median(v[group])), float(np.median(depths[group])), len(group)
        raise DetectionRejected("invalid_target_depth")

    @staticmethod
    def _depth_in_bbox(depth: np.ndarray, x1: int, y1: int,
                       x2: int, y2: int) -> Optional[float]:
        """Compute median depth in the central region of the bbox.

        Uses the inner 50% area to reduce background contamination
        from bbox edges.  Requires ≥30% valid depth pixels.
        Returns None if insufficient valid depth is found.
        """
        h, w = depth.shape
        x1 = max(0, min(x1, w - 1))
        x2 = max(x1 + 1, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(y1 + 1, min(y2, h))

        # Restrict to center 50% of each dimension (25% area)
        bbox_w = x2 - x1
        bbox_h = y2 - y1
        margin_x = bbox_w // 4
        margin_y = bbox_h // 4
        cx1 = x1 + margin_x
        cx2 = max(cx1 + 1, x2 - margin_x)
        cy1 = y1 + margin_y
        cy2 = max(cy1 + 1, y2 - margin_y)

        patch = depth[cy1:cy2, cx1:cx2]
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
