# Delta Robot Project — Progress Log

**Project:** 3-axis Delta Robot with Pneumatic Gripper, CAN Bus Control, Vision-Guided Pick-and-Place  
**Platform:** ROS 2 Humble · Ubuntu 22.04 · Python 3.10  
**Hardware:** RobStride RS-00 motors (×3) · Intel RealSense D455 · Pneumatic solenoid gripper  

---

## Session Log

### 2026-04-05 — Session 1

#### Codebase State at Start
- Single initial commit (`5d2e77b`) with full ROS 2 workspace
- Modified files: `blind_pick_place.py`, `blind_pick_place.launch.py`, `setup.py`
- New file: `pick_place_ui.py`

#### Slide Presentation (2-DCLab Report Template.pptx)
Updated from 15 slides → 27 slides. New slides added:

| Slide | Content |
|---|---|
| 7 | FK/IK — Delta robot kinematics theory (sphere intersection, line-circle geometry) |
| 9 | MIT Mode — Impedance control theory (Hogan 1985), τ = Kp·Δθ + Kd·dθ/dt + τ_ff |
| 10 | PP Mode — CiA 402 position-profile, Type 18 CAN frames, MIT vs PP tradeoff |
| 13 | ROS 2 System Architecture — DDS pub/sub theory, node graph, launch sequence |
| 14 | ROS 2 Data Flow Diagram (from `diagrams/dataflow.drawio`) |
| 15 | Trapezoidal Velocity Profile — motion profile theory, 3-phase equations |
| 16 | Pick-and-Place FSM — Moore machine theory, state transitions, threading safety |
| 17 | State Flow Diagram (from `diagrams/diagrams.drawio`) |
| 18 | Pneumatic Gripper & CAN Bus — solenoid actuation theory, ISO 11898 CAN theory |
| 19 | Camera System — active stereo depth theory, D435 → D455 switch, Y-offset mount |
| 20 | Camera Data Flow Diagram (from `diagrams/camera_diagrams.drawio`) |
| 21 | Camera Processing Workflow (from `diagrams/camera_diagrams.drawio`) |
| 22 | Pick-and-Place Terminal UI (`pick_place_ui.py`) |
| 23 | Challenges — CAN timing, camera occlusion, trajectory jerk, gripper open-loop |
| 24 | Results & Discussion — achieved items, camera findings, limitations |
| 25 | (Robot diagram — top/front view) |
| 26 | Conclusion & Next Steps |

#### Camera System Progress
- **Phase 1:** Intel RealSense D435 mounted on base frame
  - Problem: IR projector light blocked by base structure → depth noise
  - Problem: Robot arms entered camera FOV during pick → occlusion
- **Phase 2:** Switched to Intel RealSense D455
  - Longer baseline (95 mm vs 50 mm) → better depth accuracy at 300–500 mm
  - Global shutter → less motion blur
  - New mount: translated along +Y from base frame (in front of robot)
  - External flashlight/LED required to illuminate pick area
  - Extrinsic calibration must be redone after repositioning

#### ROS 2 Implementation (Completed — was "Next Step" in previous report)
- `blind_pick_place.py` — FSM node: IDLE→MOVING_TO_PICK→GRASPING→MOVING_TO_PLACE→DROPPING→RESETTING→IDLE
- `pick_place_ui.py` — terminal UI for operator-in-the-loop testing
- `blind_pick_place.launch.py` — starts can_driver_node first, 8 s delay before pick node
- Trapezoidal velocity profile replacing linear interpolation (`delta_common/trajectory.py`)
- Pneumatic gripper via CAN ID 4 routed through can_driver topic

#### Build Fixes
- Added `src/python/COLCON_IGNORE` — stops colcon picking up standalone `robstride-control` package
- Fixed stale symlinks: cleared `build/` and `install/` for `delta_camera_system`, `custom_messages`, `realsense2_camera_msgs`
- Fixed `tests_require` → `extras_require={'test': [...]}` in 4 `setup.py` files:
  - `src/testing_can/setup.py`
  - `src/can_driver/can_driver/setup.py`
  - `src/realsense_yolo_detector/setup.py`
  - `src/realsense-ros/realsense2_ros_mqtt_bridge/setup.py`
- **Result:** `colcon build --symlink-install` → 14 packages finished, 0 warnings, 0 stderr

#### File Organisation
- Created `diagrams/` — all `.drawio` and `.dot` source files
- Created `slide_images/` — all exported PNG images used in slides

---

## Next Steps (as of 2026-04-06)

1. Fix flashlight placement and optimise LED illumination for D455 FOV
2. Re-calibrate camera extrinsics with new Y-offset mount (`calibrate_plane_homography.py`)
3. Validate vision-guided pick-and-place end-to-end (D455 + YOLO11 / white rectangle)
4. Add gripper feedback (force/contact sensor) for pick verification
5. Implement approach/lift Z-offset phases for safer descent onto object
6. Integrate RViz visualisation for real-time trajectory monitoring
7. Performance benchmarking: cycle time, repeatability, pick success rate

---

## Hardware Reference

| Component | Details |
|---|---|
| Motors | 3× RobStride RS-00, CAN IDs 1–3 (extended 29-bit) |
| Gripper | Pneumatic solenoid, CAN ID 4 (standard 11-bit) |
| Camera | Intel RealSense D455 (upgraded from D435) |
| CAN | SocketCAN `can0` at 1 Mbit/s |

**Kinematic parameters:**

| Symbol | Value | Description |
|---|---|---|
| `e` | 35 mm | End-effector radius |
| `f` | 157 mm | Base radius |
| `re` | 400 mm | Forearm length |
| `rf` | 200 mm | Bicep length |

**Workspace limits:** X/Y ±151.6 mm · Z −500 mm to −196.9 mm · Home: (0, 0, −300 mm)
