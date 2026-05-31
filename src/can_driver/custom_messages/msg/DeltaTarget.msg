# DeltaTarget — RealSense detection result forwarded to MATLAB IK solver
# Published on /delta/matlab/target_xyz by the matlab_bridge_node.
#
# All positions are in the robot base frame (mm).
# Z is EE-tip depth (negative = below base plane).
#
# MATLAB example:
#   sub = ros2subscriber(node, "/delta/matlab/target_xyz", "custom_messages/DeltaTarget");
#   msg = receive(sub, 5);
#   [t1,t2,t3] = my_delta_ik(msg.x_mm, msg.y_mm, msg.z_mm);

std_msgs/Header header

float64 x_mm            # target X  (robot base frame, mm)
float64 y_mm            # target Y  (robot base frame, mm)
float64 z_mm            # target Z  (EE-tip, mm — negative below base plane)

float32 confidence      # detection confidence [0.0 – 1.0]; -1 if unavailable
int32   track_id        # camera tracking ID; -1 if unavailable
string  detection_mode  # "orange_blob", "yolo", "fake", etc.
