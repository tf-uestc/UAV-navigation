# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UAV autonomous search-and-rescue navigation system integrating VLM visual understanding, EGO-Planner trajectory planning, and PX4 flight control. This is a master's thesis project running on **ROS Noetic + Gazebo Classic 11 + PX4 SITL v1.13.x** on Ubuntu 20.04.

The system implements the pipeline: **Natural language instruction → VLM target detection → 3D localization via TF → EGO-Planner trajectory → MAVROS offboard control → PX4 execution**.

## Build & Run

```bash
# Source ROS + workspace
source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash

# Build (from ws/)
cd ws && catkin_make

# Load API key
source .env   # exports VLM_API_KEY

# --- Two launch modes ---

# Mode A: VLM detection → direct setpoint (no planner, used for F2 testing)
roslaunch uav_vln_bringup vlm_navigation.launch

# Mode B: Full pipeline (VLM → planner → follower → bridge)
# Requires EGO-Planner workspace sourced first
roslaunch uav_vln_bringup vlm_navigation_full.launch
# Or the EGO integration test (no VLM, manual target):
roslaunch uav_vln_bringup test_planner_ego.launch

# Start PX4 SITL + Gazebo first (separate terminal):
WORLD=search_rescue ./scripts_start_px4_sitl_gazebo.sh

# Send commands via rostopic:
rostopic pub -1 /uav/state_cmd std_msgs/String "data: 'takeoff'"
rostopic pub -1 /uav/instruction std_msgs/String "data: '{\"target\":{\"text\":\"红色支架\"}}'"

# View debug output:
rosrun rqt_image_view rqt_image_view /uav/target_debug
```

## Architecture

### Module Contract Pattern (ABC + importlib)

All replaceable components follow the same pattern: an abstract base class defines the interface, concrete implementations live in subdirectories, and a generic ROS node loads the implementation via `importlib` at runtime using a `~xxx_class` rosparam. This means **swapping a VLM provider or planner requires zero changes to downstream code** — just edit the launch file's `xxx_class` parameter.

```
scripts/
  detection/
    base.py             # Detector ABC + Detection dataclass
    qwen_vl_grounding.py    # Qwen VL grounding via DashScope API
  planning/
    base.py             # Planner ABC + Trajectory dataclass
    ego_adapter.py      # Bridge between Planner ABC and EGO-Planner C++ node
    dummy_planner.py    # Test stub
  detector_node.py      # Generic ROS node: subscribes images, loads Detector, publishes /uav/target_world
  planner_node.py       # Generic ROS node: subscribes target, loads Planner, publishes /uav/trajectory
  trajectory_follower.py   # Interpolates trajectory → /uav/goal_pose at 30 Hz
  setpoint_bridge.py    # State machine bridging /uav/goal_pose → MAVROS offboard (do NOT modify)
  tf_healthcheck.py     # Startup TF tree validation (required="true" node)
```

### ROS Topic Contracts (key data flow)

```
detector_node     → /uav/target_world (PointStamped, map frame, latched)
planner_node      → /uav/trajectory    (nav_msgs/Path, map frame, latched)
trajectory_follower → /uav/goal_pose   (PoseStamped, ≥20 Hz, consumed by setpoint_bridge)
/uav/instruction  ← String (JSON, e.g. {"target":{"text":"红色支架"}})
/uav/state_cmd    ← String (TAKEOFF/LAND/HOVER/RTL/EMERGENCY/GOTO)
/uav/target_debug → Image (annotated detection result for rqt_image_view)
```

### TF Frame Chain (REP-105 + REP-103)

```
map → odom → base_link → camera_link → camera_link_optical
```

- `map`: global ENU frame (east-north-up), MAVROS publishes `map→base_link` and `map→odom`
- `camera_link_optical`: ROS optical convention (z-forward, x-right, y-down) — **this is the frame detection results are in**
- Static transforms `base_link→camera_link` and `camera_link→camera_link_optical` are published by `mavros_px4_sitl.launch`
- `tf_healthcheck.py` validates that `map→base_link`, `base_link→camera_link_optical`, and `map→camera_link_optical` are all available before the rest of the system starts

### setpoint_bridge.py (critical, zero-modifications)

This is the **stable flight control interface**. It implements a finite state machine (IDLE → TAKEOFF → FLYING → HOVER/LAND/RTL) and handles all MAVROS/PX4 interactions (arming, OFFBOARD mode switching, 20 Hz setpoint stream). Algorithm nodes only publish to `/uav/goal_pose` and `/uav/state_cmd`; they never interact with MAVROS directly. This file should not be modified.

### EGO-Planner Integration

EGO-Planner is a C++ ROS node, not a Python library. `EgoPlannerAdapter` communicates with it via ROS topics:
- Publishes goals to `/move_base_simple/goal` (PoseStamped)
- Subscribes to EGO's output `/planning/bspline` (custom `ego_planner/Bspline` message)
- Converts B-spline to the Trajectory dataclass for downstream consumption
- Requires `ego_planner` workspace to be sourced (for the custom message type)

### VLM Detection (QwenVLGroundingDetector)

Calls DashScope's OpenAI-compatible API with a base64-encoded JPEG image and a grounding prompt. Parses bbox from the response (supports Qwen2-VL format `<|box_start|>`, classic `<box>`, and fallback bracket patterns). Applies a normalization heuristic (coordinates may be [0,1000] or raw pixels), fuses with depth data (median depth in bbox, requires ≥30% valid pixels), and back-projects to camera optical frame coordinates.

## Key Constraints

- **Never modify**: `setpoint_bridge.py`, `scripts_start_px4_sitl_gazebo.sh`, `empty.world`
- **Don't add drone models** to world files — PX4 SITL spawns the drone externally
- **Don't add `<gui>` camera config** to world files — the drone's onboard camera is the only camera
- **API key**: stored in `.env` (gitignored), uses `export` prefix so `source .env` populates environment for `$(env VLM_API_KEY)` in launch files
- **World file usage**: `WORLD=search_rescue ./scripts_start_px4_sitl_gazebo.sh` (the script reads `WORLD` env var, defaults to `empty`)

## External Dependencies

| Component | Path | Notes |
|---|---|---|
| PX4-Autopilot | `/home/tf/PX4-Autopilot` (or `$PX4_AUTOPILOT_DIR`) | v1.13.3, started via `make px4_sitl gazebo-classic_iris_depth_camera` |
| EGO-Planner | `ws/src/ego-planner/` | C++ nodes: `ego_planner_node`, `waypoint_generator` |
| Custom drone model | `simulation/px4_gazebo_classic/models/iris_depth_camera/` | Iris quadcopter with RGB-D camera |
| Custom worlds | `simulation/px4_gazebo_classic/worlds/` | `empty.world` (fallback), `search_rescue.world` (landing pad + red tripod obstacles) |

## Simulation Startup Flow

1. `scripts_start_px4_sitl_gazebo.sh` starts roscore (if not running), sets Gazebo plugin/model paths pointing to `simulation/px4_gazebo_classic/`, then runs `make px4_sitl gazebo-classic_${MODEL}` in the PX4 directory — this spawns the drone and loads the specified world.
2. After Gazebo + PX4 are running, launch one of the ROS launch files to start the navigation pipeline.
3. Send `takeoff` command via `/uav/state_cmd`, then send a detection instruction via `/uav/instruction`.
