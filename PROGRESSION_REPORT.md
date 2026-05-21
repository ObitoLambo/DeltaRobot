# Delta Workspace Progression Report

Prepared on April 23, 2026

## Project Summary

`delta_ws` is a ROS 2 Humble workspace for a 3-axis delta robot with CAN-based motor control, a pneumatic gripper, and a RealSense vision system for blind and vision-guided pick-and-place. The workspace is organised into modular packages for common robot math, motor control, camera processing, application logic, CAN communication, and testing utilities.

At this stage, the project has moved beyond early workspace setup and now shows clear progression toward an integrated robotic application. The system already includes kinematics, trajectory generation, CAN communication, a blind pick-and-place finite state machine, operator tooling, and an evolving camera pipeline. Current work is focused on improving reliability, calibration accuracy, safer motion planning, and preparing the vision side for more robust real-world use.

## Timeline of Progress

### April 3, 2026 - Initial Workspace Foundation

The first commit (`5d2e77b`) established the base ROS 2 workspace structure. This gave the project its core technical foundation:

- `can_driver` for SocketCAN communication and custom ROS messages
- `delta_common` for configuration, forward/inverse kinematics, and trajectory utilities
- `delta_motor_controller` for motor and pneumatic gripper control
- `delta_camera_system` for RealSense-based perception
- `delta_main_app` for application-level pick-and-place logic
- supporting packages for testing, experiments, and motor integration

This phase can be described as the system architecture phase. The main achievement was getting the robot software split into reusable packages instead of one monolithic script, which makes the project easier to debug, document, and extend.

### April 21, 2026 - Application Logic and Documentation Improvement

The second commit (`020280a`) marked a transition from base setup into more complete application behaviour. This commit included:

- updates to `blind_pick_place.py`
- addition of `pick_place_ui.py` for operator interaction
- launch-file updates for easier execution
- creation of `PROGRESS.md`
- reorganisation of diagrams into a dedicated `diagrams/` folder
- package setup fixes and testing-related cleanup

This phase improved both usability and maintainability. The project was no longer only about making packages exist; it started behaving like a usable robot application with clearer workflows, cleaner documentation, and better support for demonstrations and reporting.

### April 23, 2026 - Current Local Development Snapshot

As of April 23, 2026, the working tree contains additional local changes that are not yet committed to git. These changes show important technical progression in several subsystems.

#### 1. Camera and Coordinate Transformation Improvements

Current edits in `src/delta_camera_system/delta_camera_system/camera_system.py` and `src/delta_common/delta_common/config.py` show a shift toward a more rigorous calibration model:

- camera translation values were updated to a new mount position
- the camera rotation mapping was corrected to match the current installation
- a full 4x4 homogeneous transform (`T_cam_to_base`) was introduced
- base-frame conversion now uses matrix multiplication instead of a simpler rotation-plus-translation shortcut
- performance monitoring was added through FPS and processing-time overlays
- object velocity estimation was added and published as `/delta/object_velocity_mm_s`

This is important because it shows the project moving from "camera works" toward "camera measurements are physically meaningful and usable for motion prediction."

#### 2. Workspace and Kinematics Validation

Changes in `src/delta_common/delta_common/fk_ik.py` and `src/delta_common/delta_common/config.py` improve motion safety and correctness:

- the configured `Z_MAX` was corrected from an unrealistic value to one that better matches inverse-kinematics feasibility
- workspace validation now checks inverse kinematics directly instead of trusting only a rectangular bounding box

This is a meaningful progression because it reduces the chance of commanding unreachable points, which is a common source of runtime failure in delta robots.

#### 3. Blind Pick-and-Place Behaviour Expansion

The local changes in `src/delta_main_app/delta_main_app/blind_pick_place.py` show the strongest progression on the application side:

- home position was updated
- speed presets (`low`, `medium`, `max`) were introduced
- the node now accepts camera target input and conveyor velocity input
- random pick/place cycling services were added for repeated experiments
- safer transit motion was added using intermediate waypoints at a safe Z height
- approach and placement behaviour were expanded beyond a simple direct move

These edits show the application evolving from a basic single-cycle demo into a more testable and experiment-ready pick-and-place system.

#### 4. Operator UI Improvement

The current edits in `src/delta_main_app/delta_main_app/pick_place_ui.py` improve usability:

- support for choosing single mode, random mode, or stop mode
- speed-mode selection from the terminal UI
- support for entering multiple pick and place points
- ability to trigger or stop repeated random cycles

This progression matters because it reduces the amount of manual code editing needed during experiments and makes the system easier to operate during testing sessions.

#### 5. Motor Verification and Motion Reliability

The changes in `src/delta_motor_controller/delta_motor_controller/motor_controller.py` improve execution reliability:

- motor velocity and acceleration limits are now configurable
- the post-command verification logic now polls until the end effector settles within position tolerance or a timeout is reached
- trajectory completion is no longer treated as a fixed-delay assumption

This is a practical improvement for real hardware, where overshoot, settling time, and latency often make fixed waits unreliable.

## Overall Project Progression

The development of `delta_ws` can be summarised in four stages:

1. Foundation stage: create the ROS 2 workspace, package structure, and low-level control building blocks.
2. Integration stage: connect motor control, gripper logic, kinematics, launch files, and blind pick-and-place behaviour.
3. Usability stage: add operator UI, diagrams, progress documentation, and cleaner package setup.
4. Refinement stage: improve camera calibration, workspace validation, motion safety, velocity estimation, and repeatable experiment support.

This shows healthy project progression. The workspace is no longer only a software skeleton; it is becoming a proper robotics system with increasing attention to calibration quality, operational safety, and testing workflow.

## Key Achievements So Far

- Established a modular ROS 2 workspace for the full delta robot stack
- Implemented forward and inverse kinematics plus trajectory generation
- Built CAN-based motor and gripper control infrastructure
- Added blind pick-and-place application logic with FSM-based sequencing
- Added a terminal UI for easier human-in-the-loop operation
- Integrated RealSense-based detection and base-frame target publishing
- Improved documentation with diagrams and a session-based progress log
- Began adding motion prediction and more realistic calibration handling

## Main Challenges Encountered

- depth noise and occlusion issues from the earlier camera mounting arrangement
- need to recalibrate extrinsics after moving to the D455 and changing the mount
- mismatch between simple workspace bounds and real inverse-kinematics reachability
- motor settling and trajectory verification on physical hardware
- need for stronger pick verification and end-to-end vision validation

These challenges are typical for robotics projects and indicate that the current work is already in the integration-and-validation phase rather than the early prototyping phase.

## Recommended Next Steps

- validate the new camera-to-base transform with real measurements
- test conveyor-velocity estimation against observed object motion
- run repeated trials for blind and vision-guided pick-and-place
- measure cycle time, repeatability, and pick success rate
- add object-contact or gripper-feedback sensing for pick confirmation
- document final calibration values and experimental results for future reporting

## Short Conclusion

The progression of `delta_ws` is strong and technically meaningful. Between April 3, 2026 and April 23, 2026, the project advanced from a structured ROS 2 workspace into a much more integrated delta-robot application with improved motion control, a more usable operator interface, and a more serious perception pipeline. The current uncommitted work especially shows a shift from basic functionality toward robustness, calibration accuracy, and repeatable experimentation.
