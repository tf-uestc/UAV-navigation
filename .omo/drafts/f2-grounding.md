# Draft: F2 VLM Grounding Plan

## Requirements (confirmed from spec)
- Replace VLM text "(u,v)" output → bbox grounding → 3D world coordinate
- Detector ABC + Detection dataclass (per ARCHITECTURE.md §4.1)
- QwenVLGroundingDetector as baseline implementation
- detector_node.py: generic entry, importlib load, tf2 transform, publish target_world + target_debug
- Slim down vlm_navigation.py to orchestrator role only
- New launch vlm_navigation_v2.launch, old launch preserved for rollback
- API key from env var $VLM_API_KEY (not hardcoded)
- Camera topics use /iris_depth_camera/camera/... prefix (fixed in F3)

## Technical Decisions (auto-decided, override if needed)
- **参数注入**: `~detector_args` passed as **kwargs to detector `__init__`
- **MVP 直发 goal_pose**: detector_node publishes BOTH /uav/target_world (PointStamped, for F4) AND /uav/goal_pose (PoseStamped, backward compat with setpoint_bridge)
- **Test strategy**: Agent-executed QA only (no unit test framework in this ROS project, consistent with F3 approach)
- **Dummy detector**: detection/dummy.py — returns center 100×100 box, tests node skeleton

## Scope Boundaries
- IN: 4 new files (detection/base.py, detection/qwen_vl_grounding.py, detection/dummy.py, detector_node.py) + modify vlm_navigation.py + new launch + CMakeLists.txt update + docs/secrets.md
- OUT: Multi-frame filtering, dynamic tracking, second detector impl, planner integration, old code deletion

## Reusable from vlm_navigation.py
- VLMApiClient class (lines 61-175): HTTP calls, API config, _parse_coords
- DepthProjector class (lines 180-265): CameraInfo订阅, get_depth_at, pixel_to_camera
- cv_bridge, numpy imports

## Replaced by F3
- camera_to_world_approx → tf2_buffer.transform() (F3 fixed TF chain)

## New
- detection/ directory with __init__.py
- Bbox parsing: 4 regex patterns + normalization detection (1000 vs pixel)
- Depth fusion: median over bbox region (replaces 3×3 kernel single point)
- Debug image: green bbox + red cross + text overlay
