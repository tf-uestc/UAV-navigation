# F3 Security Review — Launch Files

## Files Reviewed
- `ws/src/uav_vln_bringup/launch/mavros_px4_sitl.launch` (29 lines, was 9 in initial commit)
- `ws/src/uav_vln_bringup/launch/vlm_navigation.launch` (65 lines, was 65 in initial commit)

## Changes Identified (vs initial commit `7b1b239`)

### mavros_px4_sitl.launch — F3 changes (+20 lines)
| Line(s) | Change | Type |
|---------|--------|------|
| 10–14 | `local_position/tf/*` rosparams (send, frame_id, child_frame_id) | TF config |
| 17–19 | `global_position/tf/*` rosparams (send, frame_id, child_frame_id) | TF config |
| 21–28 | Two `static_transform_publisher` nodes (base_link→camera_link, camera_link→camera_link_optical) | TF config |

### vlm_navigation.launch — camera topic updates (+2 lines changed)
| Line(s) | Change | Type |
|---------|--------|------|
| 48 (old) | `camera_topic_rgb`: `/camera/rgb/image_raw` | — |
| 48 (new) | `camera_topic_rgb`: `/iris_depth_camera/camera/rgb/image_raw` | Topic name |
| 49 (old) | `camera_topic_depth`: `/camera/depth/image_raw` | — |
| 49 (new) | `camera_topic_depth`: `/iris_depth_camera/camera/depth/image_raw` | Topic name |

## Security Check Results

### 1. New Secrets Introduced?
**NO.** All 20 lines added to `mavros_px4_sitl.launch` are pure TF configuration — ROS parameter overrides and `static_transform_publisher` nodes. No strings resembling API keys, tokens, passwords, or credentials.

### 2. Existing API Key Touched?
**NO.** The pre-existing API key at `vlm_navigation.launch:43` (`vlm_api_key = sk-faeac5f4c6794a88b8f631c003d429ed`) was present in the initial commit and remains **unchanged** — same line, same value, same context. The F3 changes did not go near line 43.

### 3. Unsafe Parameter Changes?
**NO.** The changed parameters are:
- `local_position/tf/*` — frame transform configuration (standard MAVROS params)
- `global_position/tf/*` — frame transform configuration
- Static transforms — 0.1m offset and standard ROS optical-frame rotation
- Camera topic names — IPC topic strings, not secrets

None of these are authentication-related, expose credentials, or weaken security posture.

### 4. Sensitive Info in Comments?
**NO.** Comments describe:
- TF chain architecture (map→odom→base_link→camera_link→camera_link_optical)
- SDF pose references
- ROS optical-frame convention
No credentials, keys, or secrets in any comment.

### 5. Changes Scoped Only to TF Config and Topic Names?
**YES.** Confirmed. Zero lines touch `vlm_api_key`, `vlm_provider`, `fcu_url`, or any authentication/authorization configuration.

---

## VERDICT: ✅ APPROVE

**No security issues introduced.** Changes are limited to:
- MAVROS TF `send`/`frame_id`/`child_frame_id` rosparam overrides
- Two `static_transform_publisher` nodes
- Camera topic name strings (`/iris_depth_camera/camera/...`)

All changes are configuration-only, security-neutral, and do not alter any credential, authentication, or authorization surface.
