#!/usr/bin/env python3
"""
pick_place_ui.py — Interactive terminal UI for blind pick-and-place.

Prompts for pick and place coordinates, updates the blind_pick_place node
parameters, then triggers the sequence.

Usage:
    ros2 run delta_main_app pick_place_ui
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PointStamped


class PickPlaceUI(Node):

    def __init__(self):
        super().__init__('pick_place_ui')
        self._pub = self.create_publisher(PointStamped, '/delta/blind_target', 10)
        self._set_param_cli = self.create_client(
            SetParameters, '/blind_pick_place/set_parameters')

    def _ask(self, label: str, default: float) -> float:
        while True:
            raw = input(f'  {label} [{default}]: ').strip()
            if raw == '':
                return default
            try:
                return float(raw)
            except ValueError:
                print('    Enter a number.')

    def _set_place(self, dx: float, dy: float, dz: float) -> bool:
        if not self._set_param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('blind_pick_place not running — place params not updated')
            return False

        def make_param(name, value):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            return p

        req = SetParameters.Request()
        req.parameters = [
            make_param('drop_x', dx),
            make_param('drop_y', dy),
            make_param('drop_z', dz),
        ]
        future = self._set_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.done() and future.result() is not None

    def run(self):
        print('\n=== Delta Robot Pick-and-Place UI ===')
        print('Press Enter to keep the default value. Ctrl+C to exit.\n')

        while rclpy.ok():
            try:
                print('-- Pick position (mm) --')
                px = self._ask('pick_x', 0.0)
                py = self._ask('pick_y', 50.0)
                pz = self._ask('pick_z', -350.0)

                print('\n-- Place position (mm) --')
                dx = self._ask('place_x', 0.0)
                dy = self._ask('place_y', -80.0)
                dz = self._ask('place_z', -300.0)

                print(f'\n  Pick  : ({px}, {py}, {pz})')
                print(f'  Place : ({dx}, {dy}, {dz})')
                if input('Send? [Y/n]: ').strip().lower() in ('n', 'no'):
                    print('Cancelled.\n')
                    continue

                # Push place coords to blind_pick_place node
                self._set_place(dx, dy, dz)

                # Publish pick target — triggers the sequence
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base'
                msg.point.x = px
                msg.point.y = py
                msg.point.z = pz
                self._pub.publish(msg)

                print('Sequence triggered — watch /delta/robot_state\n')

            except (KeyboardInterrupt, EOFError):
                print('\nExiting.')
                break


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceUI()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
