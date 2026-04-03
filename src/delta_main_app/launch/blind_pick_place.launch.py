"""
blind_pick_place.launch.py

Starts the CAN driver (brings up can0, owns the solenoid/gripper) and then
the blind pick-and-place node.  A 3-second delay is added before the pick
node so that can_driver has time to bring up the CAN interface before
DeltaMotorController opens its RobstrideBus socket.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    can_driver = Node(
        package='can_driver',
        executable='can_driver_node',
        name='can_driver_node',
        output='screen',
    )

    blind_pick_place = Node(
        package='delta_main_app',
        executable='blind_pick_place',
        name='blind_pick_place',
        output='screen',
    )

    return LaunchDescription([
        can_driver,
        # Wait for can_driver to bring up can0 before the motor controller
        # tries to open its own socketcan socket on the same interface.
        TimerAction(period=3.0, actions=[blind_pick_place]),
    ])
