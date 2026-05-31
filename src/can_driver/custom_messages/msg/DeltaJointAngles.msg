# DeltaJointAngles — IK solution from MATLAB, received by matlab_bridge_node.
# MATLAB publishes this on /delta/matlab/joint_thetas.
# bridge_node reads ik_valid first; if False the move is skipped.
#
# MATLAB example:
#   pub  = ros2publisher(node, "/delta/matlab/joint_thetas", "custom_messages/DeltaJointAngles");
#   msg  = ros2message("custom_messages/DeltaJointAngles");
#   msg.theta1_deg = t1;  msg.theta2_deg = t2;  msg.theta3_deg = t3;
#   msg.ik_valid   = true;
#   send(pub, msg);

std_msgs/Header header

float64 theta1_deg      # motor 1 joint angle (degrees, must be ≥ 0)
float64 theta2_deg      # motor 2 joint angle (degrees, must be ≥ 0)
float64 theta3_deg      # motor 3 joint angle (degrees, must be ≥ 0)

bool    ik_valid        # True = valid IK solution; False = MATLAB could not solve
