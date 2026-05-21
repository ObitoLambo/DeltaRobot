#!/usr/bin/env python3
"""
pick_place_ui.py — Interactive terminal UI for blind pick-and-place.

Prompts for pick and place coordinates, updates the blind_pick_place node
parameters, then triggers the sequence.  Supports a random mode where you
define multiple pick and place points and the node cycles through them randomly.

Usage:
    ros2 run delta_main_app pick_place_ui
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty


class PickPlaceUI(Node):

    def __init__(self):
        super().__init__('pick_place_ui')
        self._pub = self.create_publisher(PointStamped, '/delta/blind_target', 10)
        self._set_param_cli = self.create_client(
            SetParameters, '/blind_pick_place/set_parameters')
        self._random_cli = self.create_client(Empty, '/delta/trigger_random')
        self._stop_cli   = self.create_client(Empty, '/delta/stop_random')

    # ── input helpers ─────────────────────────────────────────────────────────

    def _ask(self, label: str, default: float) -> float:
        while True:
            raw = input(f'  {label} [{default}]: ').strip()
            if raw == '':
                return default
            try:
                return float(raw)
            except ValueError:
                print('    Enter a number.')

    def _ask_speed_mode(self, current: str) -> str:
        options = ('low', 'medium', 'max')
        while True:
            raw = input(f'  speed_mode {list(options)} [{current}]: ').strip().lower()
            if raw == '':
                return current
            if raw in options:
                return raw
            print(f'    Choose from {list(options)}.')

    def _ask_mode(self) -> str:
        options = ('single', 'random', 'conveyor', 'stop')
        while True:
            raw = input(f'\nMode {list(options)}: ').strip().lower()
            if raw in options:
                return raw
            print(f'    Choose from {list(options)}.')

    def _collect_points(self, label: str, defaults) -> list:
        points = []
        dx, dy, dz = defaults
        print(f'\n-- {label} points (enter blank line to finish) --')
        idx = 1
        while True:
            raw = input(f'  Add point {idx}? [Y/n]: ').strip().lower()
            if raw in ('n', 'no'):
                break
            x = self._ask(f'  {label.lower()}_{idx} x', dx)
            y = self._ask(f'  {label.lower()}_{idx} y', dy)
            z = self._ask(f'  {label.lower()}_{idx} z', dz)
            points.append((x, y, z))
            idx += 1
            if idx > 10:
                print('  (max 10 points reached)')
                break
        return points

    # ── ROS helpers ───────────────────────────────────────────────────────────

    def _set_params(self, dx: float, dy: float, dz: float, mode: str) -> bool:
        if not self._set_param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('blind_pick_place not running — params not updated')
            return False

        def make_double(name, value):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            return p

        def make_string(name, value):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=value)
            return p

        req = SetParameters.Request()
        req.parameters = [
            make_double('drop_x', dx),
            make_double('drop_y', dy),
            make_double('drop_z', dz),
            make_string('speed_mode', mode),
        ]
        future = self._set_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.done() and future.result() is not None

    def _call_empty(self, client, name: str):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'{name} service not available')
            return
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def _push_random_points(self, pick_pts, place_pts):
        """Send pick/place point lists directly to the node's internal lists."""
        if not self._set_param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('blind_pick_place not running')
            return False

        flat_pick  = [c for pt in pick_pts  for c in pt]
        flat_place = [c for pt in place_pts for c in pt]

        def make_double_array(name, values):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=values)
            return p

        req = SetParameters.Request()
        req.parameters = [
            make_double_array('random_pick_points',  flat_pick),
            make_double_array('random_place_points', flat_place),
        ]
        future = self._set_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.done() and future.result() is not None

    # ── main loop ─────────────────────────────────────────────────────────────

    def run(self):
        print('\n=== Delta Robot Pick-and-Place UI ===')
        print('Press Enter to keep defaults. Ctrl+C to exit.\n')

        speed_mode = 'low'

        while rclpy.ok():
            try:
                mode = self._ask_mode()

                # ── stop random cycling ───────────────────────────────────
                if mode == 'stop':
                    self._call_empty(self._stop_cli, '/delta/stop_random')
                    print('Stop signal sent.\n')
                    continue

                # ── single pick-and-place ─────────────────────────────────
                if mode == 'single':
                    print('\n-- Pick position (mm) --')
                    px = self._ask('pick_x',  0.0)
                    py = self._ask('pick_y',  50.0)
                    pz = self._ask('pick_z', -350.0)

                    print('\n-- Place position (mm) --')
                    dx = self._ask('place_x',  0.0)
                    dy = self._ask('place_y', -80.0)
                    dz = self._ask('place_z', -300.0)

                    print('\n-- Speed --')
                    speed_mode = self._ask_speed_mode(speed_mode)

                    print(f'\n  Pick  : ({px}, {py}, {pz})')
                    print(f'  Place : ({dx}, {dy}, {dz})')
                    print(f'  Speed : {speed_mode}')
                    if input('Send? [Y/n]: ').strip().lower() in ('n', 'no'):
                        print('Cancelled.\n')
                        continue

                    self._set_params(dx, dy, dz, speed_mode)

                    msg = PointStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base'
                    msg.point.x = px
                    msg.point.y = py
                    msg.point.z = pz
                    self._pub.publish(msg)
                    print('Sequence triggered — watch /delta/robot_state\n')

                # ── random cycling ────────────────────────────────────────
                elif mode == 'random':
                    pick_pts  = self._collect_points('Pick',  ( 0.0,  50.0, -350.0))
                    place_pts = self._collect_points('Place', ( 0.0, -80.0, -300.0))

                    if not pick_pts or not place_pts:
                        print('  Need at least one pick and one place point.\n')
                        continue

                    print('\n-- Speed --')
                    speed_mode = self._ask_speed_mode(speed_mode)

                    print(f'\n  Pick  points ({len(pick_pts)}): {pick_pts}')
                    print(f'  Place points ({len(place_pts)}): {place_pts}')
                    print(f'  Speed : {speed_mode}')
                    if input('Start random cycling? [Y/n]: ').strip().lower() in ('n', 'no'):
                        print('Cancelled.\n')
                        continue

                    self._push_random_points(pick_pts, place_pts)
                    self._set_params(0.0, 0.0, 0.0, speed_mode)
                    self._call_empty(self._random_cli, '/delta/trigger_random')
                    print('Random cycling started — type "stop" next to halt.\n')

                # ── conveyor mode ─────────────────────────────────────────
                elif mode == 'conveyor':
                    print(
                        '\n-- Conveyor mode --'
                        '\n   The camera detects objects on the belt and triggers'
                        '\n   picks automatically. Set the fixed drop (bin) position below.'
                    )
                    print('\n-- Drop / bin position (mm) --')
                    dx = self._ask('place_x',   0.0)
                    dy = self._ask('place_y', -80.0)
                    dz = self._ask('place_z', -300.0)

                    print('\n-- Speed --')
                    speed_mode = self._ask_speed_mode(speed_mode)

                    print(f'\n  Drop  : ({dx}, {dy}, {dz})')
                    print(f'  Speed : {speed_mode}')
                    if input('Activate conveyor mode? [Y/n]: ').strip().lower() in ('n', 'no'):
                        print('Cancelled.\n')
                        continue

                    ok = self._set_params(dx, dy, dz, speed_mode)
                    if ok:
                        print(
                            '\nConveyor mode active.'
                            '\n  - Camera will auto-trigger picks when it detects an object.'
                            '\n  - Watch /delta/robot_state for cycle status.'
                            '\n  - Type "stop" here to halt, or Ctrl+C to exit the UI.\n'
                        )
                    else:
                        print('  Warning: could not reach blind_pick_place node — is it running?\n')


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
