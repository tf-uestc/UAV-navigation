# F1 Oracle Review: TF Chain Fix

**Date**: 2026-05-31
**Reviewer**: Oracle Agent
**Status**: **APPROVE** (with 1 noted discrepancy)

---

## 1. Verdict: APPROVE

All implementation changes are correct and complete with respect to the plan's explicit instructions. The fix will resolve the TF tree disconnect for `map → base_link → camera_link → camera_link_optical` and will make camera topics accessible under the correct prefix. All acceptance criteria are expected to pass.

---

## 2. Verification Items (Per Plan Requirements)

### 2.1 rosparam Placements — AFTER `</include>`

| File | Line(s) | Check | Result |
|------|---------|-------|--------|
| `mavros_px4_sitl.launch` | 5-8 | `<include file="...px4.launch">` wraps inner config load | ✓ |
| `mavros_px4_sitl.launch` | 12-19 | Both `local_position` and `global_position` rosparam AFTER `</include>` at line 8 | ✓ |

**Execution order verified**: `px4.launch` → `node.launch` processes `<node>` containing `<rosparam command="load" file="...px4_config.yaml" />` FIRST (sets `tf/send=false` defaults), THEN lines 12-19 rosparam tags override to `true`. The `<rosparam>` placement after `</include>` ensures the override happens after the YAML load. The mavros node initializes asynchronously and reads parameters after launch parsing completes → parameters will be set before node reads them.

### 2.2 Param Paths for MAVROS Namespace

| rosparam path | Expected in px4_config.yaml (line) | Match |
|---------------|--------------------------------------|-------|
| `/mavros/local_position/tf/send` | local_position.tf.send (line 72) | ✓ |
| `/mavros/local_position/tf/frame_id` | local_position.tf.frame_id (line 73) | ✓ |
| `/mavros/local_position/tf/child_frame_id` | local_position.tf.child_frame_id (line 74) | ✓ |
| `/mavros/global_position/tf/send` | global_position.tf.send (line 54) | ✓ |
| `/mavros/global_position/tf/frame_id` | global_position.tf.frame_id (line 55) | ✓ |
| `/mavros/global_position/tf/child_frame_id` | global_position.tf.child_frame_id (line 57) | ✓ |

All paths are correct absolute paths into MAVROS's private namespace (node `name="mavros"` in global `/` scope).

### 2.3 Camera Topic Names

| File | Line | Value | SDF Reference | Match |
|------|------|-------|---------------|-------|
| `vlm_navigation.launch` | 48 | `/iris_depth_camera/camera/rgb/image_raw` | depth_camera plugin: `cameraName="camera"`, `robotNamespace=""`, `imageTopicName="rgb/image_raw"`, parent model=`iris_depth_camera` | ✓ |
| `vlm_navigation.launch` | 49 | `/iris_depth_camera/camera/depth/image_raw` | Same plugin: `depthImageTopicName="depth/image_raw"` | ✓ |

**SDF evidence** (`/home/tf/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/depth_camera/depth_camera.sdf`, lines 41-42, 47, 49):
- `<cameraName>camera</cameraName>`
- `<robotNamespace></robotNamespace>` (empty → inherits from parent model `iris_depth_camera`)
- Gazebo ROS plugin publishes as `/iris_depth_camera/camera/...` per standard namespace resolution.

### 2.4 `subst_value` Attribute Usage

| Location | Attribute | Body | Interpretation | Correct? |
|----------|-----------|------|---------------|----------|
| Line 12 | `subst_value="True"` | `true` | YAML boolean `true` after substitution eval | ✓ |
| Line 13 | `subst_value="False"` | `map` | Literal string `"map"` | ✓ |
| Line 17 | `subst_value="True"` | `true` | YAML boolean `true` | ✓ |
| Line 18 | `subst_value="False"` | `map` | Literal string `"map"` | ✓ |
| Line 19 | `subst_value="False"` | `odom` | Literal string `"odom"` | ✓ |

All correct. `subst_value="True"` used only for boolean values (no substitution args present — body is used as-is and interpreted as YAML). `subst_value="False"` used for string literals.

### 2.5 `global_position` Plugin Blacklist

`/opt/ros/noetic/share/mavros/launch/px4_pluginlists.yaml` blacklists:
```
plugin_blacklist:
- safety_area
- image_pub
- vibration
- distance_sensor
- rangefinder
- wheel_odometry
```

`global_position` is **NOT** in the blacklist. ✓ Plugin will load and TF will be broadcast when `tf/send=true`.

### 2.6 Will This Fix the TF Disconnect?

**Yes.** The implementation will create the following TF tree:

```
map
├── odom                    (from global_position: frame_id=map, child_frame_id=odom)
└── base_link               (from local_position:  frame_id=map, child_frame_id=base_link)
    └── camera_link         (static: 0.1m forward)
        └── camera_link_optical  (static: FLU→RDF rotation)
```

Acceptance criterion check:
- `tf_echo map camera_link_optical` → resolves `map→base_link→camera_link→camera_link_optical` → **SUCCESS** ✓
- `tf_echo map odom` → resolves `map→odom` via global_position → **SUCCESS** ✓
- `/mavros/local_position/tf/send` = `true` ✓
- Camera topics at `/iris_depth_camera/camera/...` match Gazebo output ✓

---

## 3. Noted Discrepancy: Chain vs Fork

### Issue

The plan states the **desired TF chain** as:
> `map → odom → base_link → camera_link → camera_link_optical`

But the implementation (following the plan's own section 1 instructions) produces a **forked tree** (see diagram above), not a chain. The critical difference:

| Aspect | Plan's chain goal | Implementation |
|--------|-------------------|----------------|
| local_position frame_id | `odom` (REP-105 convention) | `map` |
| local_position → child | `base_link` | `base_link` ✓ |
| global_position frame_id | `map` | `map` ✓ |
| global_position → child | `odom` | `odom` ✓ |
| odom → base_link edge | **YES** (via local_position) | **NO** |

The forked tree has `odom` as a sibling of `base_link` (not parent). There is **no `odom → base_link`** transform published by anything.

### Impact Assessment

- **No existing code depends on `odom`**: `setpoint_bridge.py`, `vlm_navigation.py`, `takeoff_land.py` all use `frame_id="map"` directly (verified via grep). No subscriptions or lookups target the `odom` frame.
- **No immediate functional breakage**: All acceptance criteria pass. Camera TF chain is reachable.
- **Future risk**: If EGO-Planner or other nodes expect the standard REP-105 chain (`map→odom→base_link`), they will fail to resolve transforms through `odom`. But the launch file's scope note says EGO-Planner is OUT of scope.

### Recommendation

This is a **non-blocking note**, not a rejection. Two options to align with the plan's stated chain goal:

1. **Leave as-is** (recommended): The forked tree works for all current use cases. The orphaned `odom` frame serves as a GPS reference if needed later.
2. **Fix to chain**: Change local_position frame_id from `map` to `odom` at line 13. This produces the true REP-105 `map→odom→base_link→...` chain. However, this requires verifying that MAVROS `local_position` plugin can use `odom` as frame_id without issues.

---

## 4. Complete Checklist

| # | Check | Result |
|---|-------|--------|
| F1.1 | rosparam AFTER `</include>` — execution order correct | ✓ PASS |
| F1.2 | Param paths match `/mavros/...` namespace | ✓ PASS |
| F1.3 | `subst_value` correct (True for bool, False for str) | ✓ PASS |
| F1.4 | `global_position` NOT in plugin blacklist | ✓ PASS |
| F1.5 | Camera topic prefix `/iris_depth_camera/camera/...` matches SDF | ✓ PASS |
| F1.6 | Static transforms: camera offset 0.1m, optical rotation correct | ✓ PASS |
| F1.7 | `clear_params="true"` doesn't clobber overrides (seq: clear→load→override) | ✓ PASS |
| F1.8 | `frameName="camera_link"` in SDF matches static TF child_frame_id | ✓ PASS |
| F1.9 | Plan's chain goal vs forked tree discrepancy | ⚠ NOTED |
| F1.10 | Launch XML syntax valid (self-closing `<rosparam/>`, proper nesting) | ✓ PASS |

## 5. Final Verdict

**APPROVE** — proceed to F2 (code review) and F4 (hands-on verification).

The one noted discrepancy (fork vs chain) does not block implementation. Verify actual behavior with `rosrun tf tf_echo map camera_link_optical` during F4.
