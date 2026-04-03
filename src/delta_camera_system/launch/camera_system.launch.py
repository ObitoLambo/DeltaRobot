# CHANGES: [D455 default] disable accel/gyro IMU, set conservative 640x480x15 profiles
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    align_depth_enable = LaunchConfiguration("align_depth_enable")
    enable_accel = LaunchConfiguration("enable_accel")
    enable_gyro = LaunchConfiguration("enable_gyro")
    unite_imu_method = LaunchConfiguration("unite_imu_method")

    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        parameters=[{
            "align_depth.enable": align_depth_enable,
            "rgb_camera.color_profile": color_profile,
            "depth_module.depth_profile": depth_profile,
            "enable_accel": enable_accel,
            "enable_gyro": enable_gyro,
            "unite_imu_method": unite_imu_method,
        }],
        output="screen",
    )

    camera_node = Node(
        package="delta_camera_system",
        executable="camera_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("align_depth_enable", default_value="true"),
        DeclareLaunchArgument("color_profile", default_value="640x480x15"),
        DeclareLaunchArgument("depth_profile", default_value="640x480x15"),
        DeclareLaunchArgument("enable_accel", default_value="false"),
        DeclareLaunchArgument("enable_gyro", default_value="false"),
        DeclareLaunchArgument("unite_imu_method", default_value="0"),
        realsense,
        camera_node,
    ])
