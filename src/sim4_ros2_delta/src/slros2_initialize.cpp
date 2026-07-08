// Copyright 2022-2024 The MathWorks, Inc.
// Generated 02-Jul-2026 13:41:50
#include "slros2_initialize.h"
const std::string SLROSNodeName("sim4_ROS2_delta");
// sim4_ROS2_delta/Publish1
SimulinkPublisher<custom_messages::msg::DeltaJointAngles,SL_Bus_custom_messages_DeltaJointAngles> Pub_sim4_ROS2_delta_72;
// sim4_ROS2_delta/Subscribe2
SimulinkSubscriber<custom_messages::msg::DeltaTarget,SL_Bus_custom_messages_DeltaTarget> Sub_sim4_ROS2_delta_75;
// sim4_ROS2_delta/Subscribe3
SimulinkSubscriber<geometry_msgs::msg::PointStamped,SL_Bus_geometry_msgs_PointStamped> Sub_sim4_ROS2_delta_76;
