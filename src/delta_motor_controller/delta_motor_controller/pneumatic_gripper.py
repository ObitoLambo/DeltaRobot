#!/usr/bin/env python3
"""
pneumatic_gripper.py — Pneumatic solenoid gripper via can_driver ROS topic.

Publishes DigitalAndSolenoidCommand to /publish_digital_solenoid which is
consumed by can_driver_node.  No direct CAN access — can_driver owns the bus.
"""

import time

from custom_messages.msg import DigitalAndSolenoidCommand


class PneumaticGripper:
    """
    Gripper controller that routes commands through can_driver.

    Parameters
    ----------
    node            : rclpy.node.Node — parent ROS 2 node (provides publisher)
    can_id          : int             — CAN ID of the solenoid board (default 4)
    close_settle_s  : float           — seconds to wait after closing (default 0.5)
    open_settle_s   : float           — seconds to wait after opening  (default 0.3)
    """

    def __init__(self, node, can_id: int = 4,
                 close_settle_s: float = 0.5,
                 open_settle_s: float = 0.3):
        self._node = node
        self._can_id = can_id
        self._close_settle_s = close_settle_s
        self._open_settle_s = open_settle_s
        self._publisher = None

    # ── lifecycle ──────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Create the publisher and publish an initial open command (safe state)."""
        self._publisher = self._node.create_publisher(
            DigitalAndSolenoidCommand,
            '/publish_digital_solenoid',
            10,
        )
        self._publish(closed=False)

    def disconnect(self) -> None:
        """Open the gripper (safety release) and tear down the publisher."""
        if self._publisher is not None:
            self._publish(closed=False)
            self._node.destroy_publisher(self._publisher)
            self._publisher = None

    # ── control ────────────────────────────────────────────────────────────────

    def close(self, wait: bool = True) -> None:
        """Energise the solenoid (grip).

        Parameters
        ----------
        wait : bool
            If True, block for close_settle_s to allow pneumatics to actuate.
        """
        self._publish(closed=True)
        if wait:
            time.sleep(self._close_settle_s)

    def open(self, wait: bool = True) -> None:
        """De-energise the solenoid (release).

        Parameters
        ----------
        wait : bool
            If True, block for open_settle_s to allow pneumatics to retract.
        """
        self._publish(closed=False)
        if wait:
            time.sleep(self._open_settle_s)

    # ── internal ───────────────────────────────────────────────────────────────

    def _publish(self, closed: bool) -> None:
        if self._publisher is None:
            self._node.get_logger().warn(
                'PneumaticGripper: publish called before connect()')
            return
        msg = DigitalAndSolenoidCommand()
        msg.can_id = self._can_id
        msg.solenoid1_value = closed
        self._publisher.publish(msg)
