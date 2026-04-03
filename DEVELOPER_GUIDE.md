# Developer Guide — Delta Robot Workspace

This guide is for developers continuing work on this codebase. It assumes you have already followed the [README](README.md) installation steps.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [CAN Bus Layer](#2-can-bus-layer)
3. [Motor Controller](#3-motor-controller)
4. [Pneumatic Gripper](#4-pneumatic-gripper)
5. [Pick-and-Place FSM](#5-pick-and-place-fsm)
6. [Vision System](#6-vision-system)
7. [Custom ROS Messages](#7-custom-ros-messages)
8. [delta_common — Shared Library](#8-delta_common--shared-library)
9. [Adding a New Pick Mode](#9-adding-a-new-pick-mode)
10. [Testing Without Hardware](#10-testing-without-hardware)
11. [Calibration Procedures](#11-calibration-procedures)
12. [Build & Rebuild Workflow](#12-build--rebuild-workflow)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Node Graph                            │
│                                                                      │
│  ┌─────────────────┐      /publish_digital_solenoid                  │
│  │  blind_pick_place│ ─────────────────────────────────────┐         │
│  │  (main_app)     │                                       ▼         │
│  │                 │      /publish_motor              ┌──────────┐   │
│  │  DeltaMotor     │ ─────── (not used) ──────────►  │can_driver│   │
│  │  Controller     │                                  │   node   │   │
│  │  (RobstrideBus) │◄────── /encoder_feedback ──────  │          │   │
│  └─────────────────┘        /digital_analog_feedback  └────┬─────┘   │
│                                                            │         │
└────────────────────────────────────────────────────────────┼─────────┘
                                                             │
                                                    SocketCAN│(can0)
                                           ┌─────────────────┴──────────┐
                                           │      CAN Bus 1 Mbit/s      │
                                           │                            │
                                           │  [Motor 1] [Motor 2] [Motor 3]  ← RobStride RS-00
                                           │  [Solenoid Board CAN ID 4]      ← Pneumatic gripper
                                           │  [Sensor Board CAN ID 500]      ← Analog inputs
                                           └────────────────────────────┘
```

### Key design decisions

| Decision | Reason |
|---|---|
| `can_driver_node` is the sole owner of CAN0 setup | Centralised interface bring-up; solenoid/sensor traffic goes through ROS topics |
| `RobstrideBus` opens its own socketcan socket | Robstride uses a proprietary extended-ID protocol not handled by `can_driver_node`; SocketCAN allows multiple readers on the same interface |
| 3-second `TimerAction` in launch file | Ensures `can0` is up before `DeltaMotorController` opens its socket |
| `PneumaticGripper` publishes ROS msgs | Gripper control goes through `can_driver_node` — no second CAN socket needed |

---

## 2. CAN Bus Layer

### `can_driver/can_driver/can_driver.py`

**Class:** `CanDriver(Node, can.Listener)`

Responsibilities:
- Calls `sudo ip link set can0 up type can bitrate 1000000` on startup
- Opens a `python-can` SocketCAN bus
- Routes inbound CAN frames (standard ID only) to ROS publishers
- Routes outbound ROS messages to CAN frames

**Inbound frame routing:**

| Arbitration ID | Published topic | Message type |
|---|---|---|
| 100 – 199 | `/encoder_feedback` | `EncoderFeedback` |
| 500 – 510 | `/digital_analog_feedback` | `DigitalAndAnalogFeedback` |

**Outbound message callbacks:**

| Subscribed topic | CAN frame built by |
|---|---|
| `/publish_motor` | `motor_command_callback` |
| `/publish_servo` | `servo_command_callback` |
| `/publish_pwm` | `pwm_command_callback` |
| `/publish_digital_solenoid` | `digital_and_solenoid_command_callback` |

**Adding a new CAN message type:**

1. Define a `.msg` file in `can_driver/custom_messages/msg/`
2. Add it to `can_driver/custom_messages/CMakeLists.txt` (`rosidl_generate_interfaces`)
3. Import it in `can_driver.py`
4. Add a subscriber + callback for outbound, or extend `on_message_received` for inbound
5. Rebuild: `colcon build --packages-select custom_messages can_driver`

**Error recovery:**

`can_driver.py` implements automatic recovery. If a send raises an `OSError` (other than buffer-full), it calls `setup_can_interface()` which tears down and re-opens `can0`. This is rate-limited to once per second via `error_timer`.

---

## 3. Motor Controller

### `delta_motor_controller/motor_controller.py`

**Class:** `DeltaMotorController`

This is a **pure Python class** (not a ROS node). It is instantiated by the pick-and-place nodes.

```python
ctrl = DeltaMotorController(can_port="can0")
ctrl.connect()          # opens RobstrideBus, enables motors, zeros position
ctrl.move_xyz(x, y, z) # IK → write position targets → verify with FK
ctrl.shutdown()         # zeros, disables, disconnects
```

**Internals:**

| Method | What it does |
|---|---|
| `connect()` | `bus.connect()` → PP mode → enable all → init_zero |
| `setup_pp_mode()` | Sets run_mode=1 (position profile), VEL_MAX, ACC_SET |
| `move_xyz(x, y, z)` | Workspace check → IK → joint limit check → write radians → verify with FK |
| `execute_trajectory(waypoints, dt)` | Streams intermediate waypoints without FK verify; final waypoint uses `move_xyz` |
| `get_current_xyz()` | Reads mechanical positions → FK |

**RobStride protocol (via `robstride_dynamics`):**

- All frames use **extended 29-bit CAN IDs**
- ID format: `(communication_type << 24) | (extra_data << 8) | device_id`
- `CommunicationType.WRITE_PARAMETER` (18) — write register value
- `CommunicationType.READ_PARAMETER` (17) — read register value
- `ParameterType.POSITION_TARGET (0x7016)` — position setpoint in radians

**Motor IDs:** 1, 2, 3 (named `motor_1`, `motor_2`, `motor_3`)

**Tuning:**

```python
self.VEL_MAX  = 1.0   # rad/s — PP mode velocity limit
self.ACC_SET  = 2.0   # rad/s² — PP mode acceleration
self.VERIFY_DELAY = 0.30  # seconds to wait before reading back position
self.POS_TOL_MM   = 2.0   # mm — FK error threshold for move_xyz success
```

---

## 4. Pneumatic Gripper

### `delta_motor_controller/pneumatic_gripper.py`

**Class:** `PneumaticGripper`

Publishes `DigitalAndSolenoidCommand` to `/publish_digital_solenoid`, which `can_driver_node` converts to a CAN frame to the solenoid board (CAN ID 4).

```python
gripper = PneumaticGripper(node, can_id=4, close_settle_s=0.5, open_settle_s=0.3)
gripper.connect()       # creates publisher, sends open (safe state)
gripper.close(wait=True) # solenoid1=True, sleeps 0.5 s
gripper.open(wait=True)  # solenoid1=False, sleeps 0.3 s
gripper.disconnect()    # safety open, destroys publisher
```

**Solenoid mapping on the CAN board (CAN ID 4):**

| Field | Purpose |
|---|---|
| `solenoid1_value` | Gripper (True = grip) |
| `solenoid2_value` | Push cylinder (used by `valve_can_node`) |
| `solenoid3_value – solenoid6_value` | Available for future actuators |

**Tuning settle times:**

The `close_settle_s` and `open_settle_s` values represent the time for the pneumatic cylinder to physically actuate. If the gripper is missing objects or releasing too early, increase these values.

---

## 5. Pick-and-Place FSM

### Blind mode — `delta_main_app/blind_pick_place.py`

**FSM states and transitions:**

```
IDLE
 │  trigger (service /delta/trigger_pick or topic /delta/blind_target)
 ▼
APPROACHING   linear_waypoints(current → approach_point)  v_max=traj_v_max
 ▼
PICKING       linear_waypoints(approach → pick_point)     v_max=pick_v_max
 ▼
GRASPING      gripper.close(wait=True)
 ▼
LIFTING       linear_waypoints(pick → lift_point)         v_max=pick_v_max
 ▼
TRANSPORTING  linear_waypoints(lift → drop_point)         v_max=traj_v_max
 ▼
DROPPING      gripper.open(wait=True)
 ▼
RESETTING     linear_waypoints(drop → home=(0,0,-300))    v_max=traj_v_max
 ▼
IDLE
```

The sequence runs in a **daemon background thread** — the ROS spin loop stays responsive. Only one sequence can run at a time (`_busy` lock).

**Home position:** `(0, 0, -300)` mm — robot arm at rest.

**Approach / lift points** are computed relative to pick position:
```python
approach = (px, py, clamp_z(pz + approach_z_offset))
lift     = (px, py, clamp_z(pz + lift_z_offset))
```
Z is clamped to `[Z_MIN, Z_MAX]` from config.

### Vision-guided mode — `delta_main_app/main_app.py`

Adds a `CONFIRMING` state before `APPROACHING`. The camera must report a stable detection for `DETECTION_CONFIRM_FRAMES` consecutive frames within `STABLE_THRESH_X/Y/Z_MM` tolerance before the robot moves.

---

## 6. Vision System

### `delta_camera_system/camera_system.py`

Subscribes to RealSense topics and runs object detection. Publishes the detected object's 3D position in the robot base frame.

**Detection modes** (set `DETECTION_MODE` in `config.py`):

| Mode | Algorithm | Best for |
|---|---|---|
| `"white_rectangle"` | HSV threshold + contour | Light-coloured rectangular boxes |
| `"yolo"` | YOLO11 model | Arbitrary objects with trained model |

**2D → 3D pipeline:**

```
Pixel (u, v)
    │ RealSense aligned depth
    ▼
Camera frame XYZ (metres)
    │ CAMERA_DIRECT_MATRIX rotation + CAM_TX/Y/Z_MM translation
    ▼
Robot base frame XYZ (mm)
    │ check_workspace()
    ▼
Published on /delta/target_xyz (PointStamped, mm)
```

**Camera transform matrix** (`config.py`):

```python
CAMERA_DIRECT_MATRIX = (
    (1.0,  0.0,  0.0),   # camera +X  → robot +X
    (0.0, -1.0,  0.0),   # camera +Y  → robot −Y
    (0.0,  0.0, -1.0),   # camera +Z  → robot −Z
)
CAM_TX_MM, CAM_TY_MM, CAM_TZ_MM = 97.59, 11.93, 432.00
```

If the detected positions are systematically offset, adjust `CAM_TX_MM / CAM_TY_MM / CAM_TZ_MM` or use `CAM_FINE_ROLL/PITCH/YAW_DEG` for fine rotation.

---

## 7. Custom ROS Messages

Defined in `can_driver/custom_messages/msg/`. Rebuilt with:
```bash
colcon build --packages-select custom_messages
```

| Message | Fields |
|---|---|
| `MotorCommand` | `uint16 can_id`, `float32 goal`, `bool positionmode/speedmode/voltagemode/stop/reset` |
| `EncoderFeedback` | `uint16 can_id`, `float32 position`, `float32 speed` |
| `DigitalAndSolenoidCommand` | `uint16 can_id`, `bool digital1–4_value`, `bool solenoid1–6_value` |
| `DigitalAndAnalogFeedback` | `uint16 can_id`, `bool digital1–4_value`, `float32 analog1–5_value` |
| `PwmCommand` | `uint16 can_id`, `float32 pwm1–4_value` |
| `ServoCommand` | `uint16 can_id`, `float32 servo1–4_value` |
| `YoloDetection` | `int16 class_id`, `float32 x/y/width/height/confidence/distance` |
| `YoloDetectionArray` | `YoloDetection[] detections` |
| `CanMsg` | `uint16 can_id`, `uint8[8] data` |

---

## 8. delta_common — Shared Library

Not a ROS node. Imported by all other packages. Rebuild if you change it:
```bash
colcon build --packages-select delta_common
```

### `fk_ik.py`

```python
from delta_common.fk_ik import delta_calcInverse, delta_calcForward, check_workspace

# Inverse kinematics: XYZ (mm) → joint angles (degrees)
status, t1, t2, t3 = delta_calcInverse(x, y, z, e, f, re, rf)
# status == 0 means success

# Forward kinematics: joint angles (degrees) → XYZ (mm)
status, x, y, z = delta_calcForward(t1, t2, t3, e, f, re, rf)

# Workspace check (fast reject before IK)
ok = check_workspace(x, y, z)
```

### `trajectory.py`

```python
from delta_common.trajectory import linear_waypoints

waypoints = linear_waypoints(
    p0=(x0, y0, z0),   # start point mm
    p1=(x1, y1, z1),   # end point mm
    v_max=80.0,         # mm/s
    a_max=200.0,        # mm/s²
    dt=0.05,            # seconds per step
)
# returns list of (x, y, z) tuples
```

### `config.py`

Central place for **all** tunable parameters. Do not hardcode magic numbers in other modules — add a constant here and import it.

Flags useful during development:

```python
ENABLE_MOTORS = False   # disables all CAN output → safe for desk testing
AUTO_MOVE     = False   # camera detects but arm stays still
VISION_ONLY_ENABLE = True   # camera node runs without robot
PURE_CAMERA_TEST_ENABLE = True
```

---

## 9. Adding a New Pick Mode

1. Create `src/delta_main_app/delta_main_app/my_pick_mode.py`
2. Import `DeltaMotorController` and `PneumaticGripper`:
   ```python
   from delta_motor_controller.motor_controller import DeltaMotorController
   from delta_motor_controller.pneumatic_gripper import PneumaticGripper
   ```
3. Use the standard FSM pattern from `blind_pick_place.py` as a template
4. Register entry point in `src/delta_main_app/setup.py`:
   ```python
   'my_pick_mode = delta_main_app.my_pick_mode:main',
   ```
5. Add a launch file in `src/delta_main_app/launch/` that starts `can_driver_node` **before** your node (use `TimerAction(period=3.0, ...)`):
6. Register the launch file in `setup.py` under `data_files`
7. Rebuild: `colcon build --packages-select delta_main_app`

---

## 10. Testing Without Hardware

### Dry-run mode (no CAN)

```python
# In delta_common/config.py
ENABLE_MOTORS = False
```

All nodes start normally. Motor commands are logged but no CAN frames are sent.

### Test individual components

```bash
# Test solenoid/gripper
ros2 run testing_can test_solenoid_node

# Watch encoder feedback
ros2 topic echo /encoder_feedback

# Watch gripper commands
ros2 topic echo /publish_digital_solenoid

# Manually trigger gripper close
ros2 topic pub /publish_digital_solenoid custom_messages/msg/DigitalAndSolenoidCommand \
  "{can_id: 4, solenoid1_value: true}"

# Monitor raw CAN traffic
candump can0
```

### Test IK/FK without ROS

```bash
cd ~/delta_ws
python3 - <<'EOF'
from src.delta_common.delta_common.fk_ik import delta_calcInverse, delta_calcForward, e, f, re, rf
st, t1, t2, t3 = delta_calcInverse(0, 50, -350, e, f, re, rf)
print(f"IK: status={st}, angles=({t1:.2f}, {t2:.2f}, {t3:.2f}) deg")
st, x, y, z = delta_calcForward(t1, t2, t3, e, f, re, rf)
print(f"FK: status={st}, pos=({x:.2f}, {y:.2f}, {z:.2f}) mm")
EOF
```

---

## 11. Calibration Procedures

### Camera-to-robot transform

1. Place a calibration target at known robot-frame positions
2. Run: `ros2 run delta_camera_system calibrate_camera_transform`
3. The script computes the rotation matrix and translation vector
4. Update `CAMERA_DIRECT_MATRIX`, `CAM_TX_MM`, `CAM_TY_MM`, `CAM_TZ_MM` in `config.py`

### Plane homography (optional)

Used when the camera cannot resolve depth accurately (e.g., 2D-only mode):

1. Run: `ros2 run delta_camera_system calibrate_plane_homography`
2. Set `PLANE_HOMOGRAPHY_ENABLE = True` and update `PLANE_HOMOGRAPHY_MATRIX` in `config.py`

### Motor zero position

The motors home to `0 rad` on `DeltaMotorController.connect()`. If the robot arm is not at the neutral upright position when you power on, you need to physically adjust the arm or update `homing_offset` in the `calibration` dict inside `motor_controller.py`:

```python
calibration = {
    name: {"direction": 1, "homing_offset": 0.0}  # adjust offset here
    for name in self.MOTOR_NAMES
}
```

---

## 12. Build & Rebuild Workflow

```bash
# Full workspace build
cd ~/delta_ws
colcon build --symlink-install

# Build only changed packages
colcon build --packages-select can_driver custom_messages delta_motor_controller delta_main_app

# After changing custom_messages, always rebuild dependents too
colcon build --packages-select custom_messages can_driver delta_motor_controller delta_main_app

# Source after every build
source install/setup.bash

# Clean build (if something is very broken)
rm -rf build/ install/ log/
colcon build --symlink-install
```

**`--symlink-install`** is recommended during development — Python source files are symlinked so you don't need to rebuild after every Python edit (only needed for new files or `setup.py` changes).

---

## 13. Troubleshooting

### `can0` fails to come up

```bash
# Check if the CAN adapter is detected
ip link show | grep can

# Check kernel module
lsmod | grep can

# Manually bring up with verbose output
sudo ip link set can0 up type can bitrate 1000000
ip link show can0
```

### Motor doesn't move

1. Check `ENABLE_MOTORS = True` in `config.py`
2. Check motor power (the RS-00 needs 12–48 V supply)
3. Run `candump can0` and look for Robstride response frames (extended IDs)
4. Check that motor IDs are 1, 2, 3 (use `RobstrideBus.scan_channel('can0')`)

### Gripper doesn't actuate

1. Check `candump can0` — you should see frames to CAN ID 4
2. Check that `can_driver_node` is running (`ros2 node list`)
3. Manually publish: `ros2 topic pub /publish_digital_solenoid ...`
4. Check pneumatic air pressure

### IK returns non-zero status

The requested (x, y, z) position has no valid solution. Common causes:
- Z too shallow (close to base) — try more negative Z
- XY too large — stay within ±151 mm
- Joint limits exceeded even if workspace check passes — run `delta_calcInverse` manually and inspect angles

### ROS messages not found after adding a new `.msg` file

```bash
colcon build --packages-select custom_messages
source install/setup.bash
ros2 interface list | grep custom_messages
```

### `colcon build` fails on `robstride_dynamics`

This package (`src/robstride_dynamics/`) requires `tqdm` and `numpy`:
```bash
pip install tqdm numpy
```
