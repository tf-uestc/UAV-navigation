# f2-vlm-grounding — Design Notes

## T1.1: base_detector.py (2026-05-31)

### Decisions
- **`center` auto-computed**: Uses `field(init=False)` + `__post_init__` to derive `(u, v)` from `bbox` automatically. Detector implementations do NOT pass center — it's always `((x1+x2)//2, (y1+y2)//2)`.
- **`point_camera` is required**: Stored by detector implementation, NOT auto-computed by this base class.
- **Flat file in `scripts/`**: No `detection/` subpackage per decision Q3.
- **Field ordering**: Required fields before optional; `center` at end with `init=False`.

## T1.2: dummy_detector.py (2026-05-31)

### Decisions
- **`pixel_to_camera_optical` as module function**: Not a static method — simpler to test independently and reuse across detector implementations. Takes raw intrinsics (fx, fy, cx, cy) extracted from `cam_info.K` row-major 3x3 matrix.
- **Bbox clamping**: `max(0, ...)` / `min(w-1, ...)` ensures the box never exceeds image boundaries.
- **Valid depth filter**: `(region > 0) & np.isfinite(region)` — excludes zeros (sensor dropout), NaN, and Inf.
- **Confidence as valid-pixel ratio**: `valid_count / total_pixels` gives a simple quality metric between 0 and 1.
- **Float center for back-projection**: Uses `(x1+x2)/2.0` instead of `//2` to avoid integer truncation before the division in `pixel_to_camera_optical`.
- **No ROS imports**: Pure Python module, no `rospy` — works standalone.

## T2: qwen_vl_grounding.py (2026-05-31)

### Decisions
- **BGR conversion**: `detect()` receives RGB but `cv2.imencode` expects BGR → convert via `cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)` before encoding.
- **Prompt internal to `_call_api`**: Formats `DEFAULT_PROMPT.format(target=target_text)` inside the method, keeping the public API clean.
- **3 regex patterns**: Qwen2-VL box tokens → classic `<box>` tags → bracketed fallback. Tried in order, first match wins.
- **Normalization heuristic**: `max <= 1000 && (img > 1000 || area_ratio < 0.5)` triggers scale-up to pixel coords.
- **Bbox clamping**: Both `_parse_bbox` and `_depth_in_bbox` clamp to image dimensions independently for safety.
- **Depth filter**: `(~isnan) & (>0) & isfinite` — excludes NaN, zero, and Inf.
- **Confidence as valid-ratio**: `valid_pixels / bbox_area` computed directly in `detect()` rather than re-extracting from `_depth_in_bbox`.
- **K matrix extraction**: `K[0], K[4], K[2], K[5]` for fx, fy, cx, cy from row-major 3x3.
- **Center uses integer division**: `(x1+x2)//2` per task spec — 0.5px offset negligible for 3D back-projection.
- **healthcheck**: Returns True — no API ping to avoid unnecessary billing.

## T3: detector_node.py (2026-05-31)

### Decisions
- **importlib loading**: `cls_path.rsplit(".", 1)` splits `module.path.ClassName` into module and class. Works for flat files (`dummy_detector.DummyDetector`) and subpackages (`qwen_vl_grounding.QwenVLGroundingDetector`) alike — scripts directory must be on PYTHONPATH (catkin provides this).
- **API key merge strategy**: `~vlm_api_key` param read first; if empty, fallback to `VLM_API_KEY` env var. Merged into `detector_args` only if `"api_key"` not already present — explicit `~detector_args` overrides take precedence.
- **Threading via `threading.Thread`**: Detection runs in a daemon thread spawned from the instruction callback. This keeps the ROS callback queue responsive regardless of API latency. Lock protects the sensor snapshot but callbacks do single-assignment (atomic in CPython for references).
- **BGR image convention**: `cv_bridge` converts ROS images to `"bgr8"` encoding. Detectors receive BGR data — QwenVL's `cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)` must be adjusted if used with this node (detector implementations own their format conversions).
- **TF transform with 1.0s timeout**: `tf_buffer.transform()` lookup limited to `rospy.Duration(1.0)` to prevent blocking when transforms are unavailable. Catches all three `tf2_ros` exception types.
- **Debug image drawing**: Double-draw technique for text — white foreground at 2px thickness + black outline at 1px thickness for readability on any background. Failure text positioned at bottom-center with `(w-tw)//2, h-30`.
- **Graceful failure**: Outer try/except catches ALL exceptions — publishes a failure debug image if RGB data is available, logs error, continues running. Never crashes the node.
- **`~trigger_on_start` reserved**: Parameter read but unused — logged for future feature where detection triggers immediately on node startup without waiting for instruction.
- **Publishes ONLY `/uav/target_world`**: Per decision Q5, no goal_pose output. `PointStamped` with `frame_id="map"` — downstream nodes handle navigation logic.
- **No direct detector imports**: Uses importlib exclusively — no `from qwen_vl_grounding import ...` anywhere.
