# MATLAB Bridge Node — User Guide

## Overview

`matlab_bridge_node` connects the delta robot's RealSense detection pipeline to
MATLAB's IK solver.  Instead of computing joint angles inside ROS2, this node
forwards every confirmed target XYZ to MATLAB, waits for the three joint angles
back, then executes the move on the physical motors.

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Running on Ubuntu (ROS2)                     │
│                                                                     │
│  RealSense D455                                                     │
│      │ /delta/target_xyz (PointStamped)                             │
│      ▼                                                              │
│  matlab_bridge_node                                                 │
│      │  FSM: IDLE → WAITING_MATLAB → MOVING → RESETTING → IDLE     │
│      │                                                              │
│      │ /delta/matlab/target_xyz  (DeltaTarget)  ──────────────────►│
│      │                                                   MATLAB     │
│      │ /delta/matlab/joint_thetas (DeltaJointAngles) ◄────────────│
│      │                                                              │
│      ├──► /delta/matlab/bridge_state  (String)  — FSM state        │
│      └──► /delta/matlab/fk_result     (PointStamped) — FK verify   │
│                                                                     │
│  Motors  ◄── CAN bus (socketcan can0)                              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| ROS2 | Humble |
| MATLAB Robotics System Toolbox | R2022b or later |
| python-can | 4.x (for gripper direct CAN) |
| RealSense ROS2 wrapper | already in repo |

---

## Build

```bash
cd ~/delta_ws

# 1. Build custom messages (DeltaTarget + DeltaJointAngles)
colcon build --packages-select custom_messages

# 2. Build the main app (contains matlab_bridge_node)
colcon build --packages-select delta_main_app

# 3. Source the workspace
source install/setup.bash
```

---

## Run

### Step 1 — Start the camera and detection pipeline

```bash
# Terminal 1: RealSense camera + camera_system node
ros2 launch delta_main_app delta_main.launch.py
```

> The launch file starts the realsense2_camera node and delta_camera_system.
> The camera system publishes confirmed detections to `/delta/target_xyz`.

### Step 2 — Start the MATLAB bridge node

```bash
# Terminal 2
source ~/delta_ws/install/setup.bash
ros2 run delta_main_app matlab_bridge
```

You should see:
```
[matlab_bridge_node]: MatlabBridgeNode ready
  Listening : /delta/target_xyz
  → MATLAB  : /delta/matlab/target_xyz   (DeltaTarget)
  ← MATLAB  : /delta/matlab/joint_thetas (DeltaJointAngles)
  State     : /delta/matlab/bridge_state
  MATLAB timeout: 3.0 s
```

### Step 3 — Connect MATLAB

Run the MATLAB script below.  MATLAB must be on the **same network** as the
Ubuntu machine (or on the same machine) and able to reach ROS2 topics.

---

## MATLAB Setup

### A. Set the ROS2 domain ID (must match Ubuntu)

```matlab
% In MATLAB, set domain ID to match ROS_DOMAIN_ID on Ubuntu (default = 0)
setenv("ROS_DOMAIN_ID", "0")
```

### B. MATLAB IK bridge script (minimal working example)

```matlab
%% delta_matlab_bridge.m
% Minimal MATLAB IK bridge for the delta robot.
% Requires: Robotics System Toolbox (ros2node, ros2subscriber, ros2publisher)

node = ros2node("/matlab_ik_node");

sub = ros2subscriber(node, ...
    "/delta/matlab/target_xyz", ...
    "custom_messages/DeltaTarget");

pub = ros2publisher(node, ...
    "/delta/matlab/joint_thetas", ...
    "custom_messages/DeltaJointAngles");

disp("MATLAB IK bridge ready — waiting for targets...")

while true
    % Block until a target arrives (10 s timeout)
    tgt = receive(sub, 10);

    x = tgt.x_mm;
    y = tgt.y_mm;
    z = tgt.z_mm;

    fprintf("Target received: (%.1f, %.1f, %.1f) mm\n", x, y, z);

    % ── Solve IK ──────────────────────────────────────────────────────
    [t1, t2, t3, valid] = delta_ik_matlab(x, y, z);

    if valid
        fprintf("IK solved: θ1=%.2f° θ2=%.2f° θ3=%.2f°\n", t1, t2, t3);
    else
        fprintf("IK FAILED for (%.1f, %.1f, %.1f)\n", x, y, z);
    end

    % ── Publish reply ─────────────────────────────────────────────────
    reply = ros2message("custom_messages/DeltaJointAngles");
    reply.theta1_deg = t1;
    reply.theta2_deg = t2;
    reply.theta3_deg = t3;
    reply.ik_valid   = valid;

    send(pub, reply);
end
```

### C. MATLAB IK function (delta_ik_matlab.m)

This is the geometry-based analytical IK for the delta robot.
Paste this into a new file `delta_ik_matlab.m` in your MATLAB working directory.

```matlab
function [theta1, theta2, theta3, valid] = delta_ik_matlab(x0, y0, z0)
% DELTA_IK_MATLAB  Analytical inverse kinematics for the delta robot.
%
%   [t1, t2, t3, valid] = delta_ik_matlab(x0, y0, z0)
%
%   Inputs:  x0, y0, z0 — end-effector position in mm (robot base frame)
%   Outputs: theta1/2/3  — joint angles in degrees (0 to ~67°)
%            valid       — true if a solution exists within joint limits
%
%   Robot geometry (must match config.py):
%     e  = 35    mm   (end-effector triangle side / 2)
%     f  = 157   mm   (base triangle side / 2)
%     re = 400   mm   (upper arm length)
%     rf = 200   mm   (lower arm / rod length)

e  = 35;
f  = 157;
re = 400;
rf = 200;

THETA_MIN = -5;
THETA_MAX = 67;

[s1, theta1] = calc_angle_yz(x0,  y0,  z0, e, f, re, rf);
[s2, theta2] = calc_angle_yz( ...
    x0*cosd(-120) + y0*sind(-120), ...
    y0*cosd(-120) - x0*sind(-120), z0, e, f, re, rf);
[s3, theta3] = calc_angle_yz( ...
    x0*cosd(120) + y0*sind(120), ...
    y0*cosd(120) - x0*sind(120), z0, e, f, re, rf);

if s1 ~= 0 || s2 ~= 0 || s3 ~= 0
    theta1 = 0; theta2 = 0; theta3 = 0;
    valid = false;
    return
end

in_limits = theta1 >= THETA_MIN && theta1 <= THETA_MAX && ...
            theta2 >= THETA_MIN && theta2 <= THETA_MAX && ...
            theta3 >= THETA_MIN && theta3 <= THETA_MAX;

valid = in_limits;
end


function [status, theta] = calc_angle_yz(x0, y0, z0, e, f, re, rf)
    tan30 = 1 / sqrt(3);
    y1 = -0.5 * tan30 * f;
    y0 = y0 - 0.5 * tan30 * e;

    a = (x0^2 + y0^2 + z0^2 + rf^2 - re^2 - y1^2) / (2*z0);
    b = (y1 - y0) / z0;

    d = -(a + b*y1)^2 + rf*(b^2*rf + rf);
    if d < 0
        status = -1; theta = 0; return
    end

    yj = (y1 - a*b - sqrt(d)) / (b^2 + 1);
    zj = a + b*yj;
    theta = atan2d(-zj, (y1 - yj));
    if yj > y1
        theta = theta + 180;
    end
    status = 0;
end
```

---

## State Machine Reference

```
               target arrives on /delta/target_xyz
                          │
             ┌────────────▼────────────┐
             │          IDLE           │
             └────────────┬────────────┘
                          │  on_target()
                          │  → publish DeltaTarget to MATLAB
             ┌────────────▼────────────┐
             │     WAITING_MATLAB      │ ─── timeout (3 s) ──► ERROR
             └────────────┬────────────┘
                          │  DeltaJointAngles received
                          │  ik_valid = true
             ┌────────────▼────────────┐
             │         MOVING          │  (background thread)
             │  move_thetas(t1,t2,t3)  │  ← motor CAN commands
             └────────────┬────────────┘
                          │  move complete
             ┌────────────▼────────────┐
             │        RESETTING        │  (background thread)
             │  move_xyz(0, 0, -350)   │  ← return home
             └────────────┬────────────┘
                          │
             ┌────────────▼────────────┐
             │          IDLE           │  ready for next target
             └─────────────────────────┘

  ERROR path (from WAITING_MATLAB timeout or ik_valid=False):
      ERROR → wait 1 s → move home → IDLE
```

### State descriptions

| State | What is happening |
|---|---|
| `IDLE` | Waiting for a new detection from the camera |
| `WAITING_MATLAB` | `DeltaTarget` published; holding until MATLAB replies |
| `MOVING` | MATLAB's joint angles are being executed on the motors via CAN |
| `RESETTING` | Move finished; robot returning to home position (0, 0, −350 mm) |
| `ERROR` | Timeout or invalid IK — robot recovering to home before going IDLE |

Monitor state in real time:
```bash
ros2 topic echo /delta/matlab/bridge_state
```

---

## Topic Reference

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/delta/target_xyz` | Camera → Bridge | `geometry_msgs/PointStamped` | Confirmed detection from RealSense pipeline |
| `/delta/matlab/target_xyz` | Bridge → MATLAB | `custom_messages/DeltaTarget` | Enriched target forwarded to MATLAB |
| `/delta/matlab/joint_thetas` | MATLAB → Bridge | `custom_messages/DeltaJointAngles` | IK solution from MATLAB |
| `/delta/matlab/bridge_state` | Bridge → All | `std_msgs/String` | Current FSM state |
| `/delta/matlab/fk_result` | Bridge → All | `geometry_msgs/PointStamped` | FK position after each move (verification) |

---

## Custom Message Field Reference

### `DeltaTarget` (ROS2 → MATLAB)

| Field | Type | Unit | Notes |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | stamp + frame_id="robot_base" |
| `x_mm` | `float64` | mm | robot base frame X |
| `y_mm` | `float64` | mm | robot base frame Y |
| `z_mm` | `float64` | mm | EE-tip Z (negative = below base plane) |
| `confidence` | `float32` | 0–1 | Detection confidence; −1 if unavailable |
| `track_id` | `int32` | — | Camera track ID; −1 if unavailable |
| `detection_mode` | `string` | — | e.g. `"orange_blob"`, `"yolo"` |

### `DeltaJointAngles` (MATLAB → ROS2)

| Field | Type | Unit | Notes |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | stamp from MATLAB clock |
| `theta1_deg` | `float64` | degrees | Motor 1 — must be 0–67° |
| `theta2_deg` | `float64` | degrees | Motor 2 — must be 0–67° |
| `theta3_deg` | `float64` | degrees | Motor 3 — must be 0–67° |
| `ik_valid` | `bool` | — | Set `false` to skip move; node goes to ERROR |

---

## Tuning

### MATLAB timeout
Edit `MATLAB_TIMEOUT_S` at the top of `matlab_bridge_node.py` (default 3.0 s).
Increase if MATLAB IK is slow or the network has latency.

```python
# matlab_bridge_node.py line ~47
MATLAB_TIMEOUT_S = 3.0
```

### Motor speed
Controlled by `MOTOR_VEL_MAX` and `MOTOR_ACC_SET` in `config.py`.
```python
MOTOR_VEL_MAX = 3.0   # rad/s — increase for faster moves
MOTOR_ACC_SET = 5.0   # rad/s² — increase for snappier starts
```

### Z depth (EE offset)
The camera reports EE-tip Z.  The bridge converts it via `EE_OFFSET_Z_MM`.
If MATLAB's IK result consistently misses the object in Z, adjust:
```python
EE_OFFSET_Z_MM = -60.0   # in config.py — tune in ±5 mm steps
```

---

## Verify FK after a move

After every successful move the node publishes the FK-verified position:
```bash
ros2 topic echo /delta/matlab/fk_result
```
Compare `point.x/y/z` against the original `DeltaTarget.x_mm/y_mm/z_mm` to
check how accurately the motors executed MATLAB's solution.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `MATLAB timeout` in logs | MATLAB not running or wrong topic name | Confirm `ros2 topic list` shows `/delta/matlab/joint_thetas` |
| `ik_valid=False` | MATLAB IK has no solution for that XYZ | Check Z is in range [−510, −323] mm and XY within ±150 mm |
| `Joint limit reject` | MATLAB returned theta outside [−5°, 67°] | Fix MATLAB IK or add clamping |
| Robot goes to ERROR immediately | `ENABLE_MOTORS=False` in config + IK path issue | Check logs for the exact error message |
| Move overshoots | Motor velocity too high | Lower `MOTOR_VEL_MAX` in `config.py` |
| FK result far from target | `EE_OFFSET_Z_MM` wrong | Re-measure and tune in `config.py` |

---

## Important: Do NOT run alongside `main_app`

`matlab_bridge_node` and `delta_main_app` both consume `/delta/target_xyz` and
both write to the same CAN motors.  Run **one or the other**, never both at the
same time.

```bash
# Correct: bridge mode
ros2 run delta_main_app matlab_bridge

# Correct: autonomous mode (Python IK)
ros2 run delta_main_app main_app
```
