#!/usr/bin/env python3
"""Unit tests for the solenoid control node."""

import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _install_message_stubs():
    custom_messages = types.ModuleType('custom_messages')
    custom_messages_msg = types.ModuleType('custom_messages.msg')
    dc_gamepad_msgs = types.ModuleType('dc_gamepad_msgs')
    dc_gamepad_msgs_msg = types.ModuleType('dc_gamepad_msgs.msg')

    class DigitalAndSolenoidCommand:
        def __init__(self):
            self.can_id = 0
            self.solenoid1_value = False
            self.solenoid2_value = False

    class DigitalAndAnalogFeedback:
        def __init__(self, can_id=0, analog2_value=0.0):
            self.can_id = can_id
            self.analog2_value = analog2_value

    class GamePad:
        def __init__(
            self,
            button_a=False,
            previous_button_a=False,
            button_lb=False,
            previous_button_lb=False,
        ):
            self.button_a = button_a
            self.previous_button_a = previous_button_a
            self.button_lb = button_lb
            self.previous_button_lb = previous_button_lb

    custom_messages_msg.DigitalAndSolenoidCommand = DigitalAndSolenoidCommand
    custom_messages_msg.DigitalAndAnalogFeedback = DigitalAndAnalogFeedback
    dc_gamepad_msgs_msg.GamePad = GamePad

    custom_messages.msg = custom_messages_msg
    dc_gamepad_msgs.msg = dc_gamepad_msgs_msg

    sys.modules['custom_messages'] = custom_messages
    sys.modules['custom_messages.msg'] = custom_messages_msg
    sys.modules['dc_gamepad_msgs'] = dc_gamepad_msgs
    sys.modules['dc_gamepad_msgs.msg'] = dc_gamepad_msgs_msg

    return DigitalAndAnalogFeedback, GamePad


DigitalAndAnalogFeedback, GamePad = _install_message_stubs()

from delta_motor_controller.valve_can_node import SolenoidControlNode


class TestSolenoidControlNode(unittest.TestCase):
    """Validate the button and feedback driven state machine."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.mock_publisher = MagicMock()
        publisher_patch = patch.object(
            Node,
            'create_publisher',
            return_value=self.mock_publisher,
        )
        subscription_patch = patch.object(
            Node,
            'create_subscription',
            return_value=MagicMock(),
        )
        self.addCleanup(publisher_patch.stop)
        self.addCleanup(subscription_patch.stop)
        publisher_patch.start()
        subscription_patch.start()

        self.node = SolenoidControlNode()

    def tearDown(self):
        self.node.destroy_node()

    def _last_published(self):
        return self.mock_publisher.publish.call_args[0][0]

    def test_node_starts_with_safe_outputs(self):
        self.assertEqual(self.node.get_name(), 'valve_can_node')
        self.assertFalse(self.node.solenoid_push)
        self.assertFalse(self.node.solenoid_hand)

        last_message = self._last_published()
        self.assertEqual(last_message.can_id, self.node.PUBLISH_CAN_ID)
        self.assertFalse(last_message.solenoid1_value)
        self.assertFalse(last_message.solenoid2_value)

    def test_button_a_toggles_hand_and_cancels_cycle(self):
        self.node.dribbling = True
        self.node.solenoid_push = True
        self.node.counter_loop = 12

        self.node.gamepad_callback(GamePad(button_a=True))

        self.assertFalse(self.node.dribbling)
        self.assertFalse(self.node.solenoid_push)
        self.assertEqual(self.node.counter_loop, 0)
        self.assertTrue(self.node.solenoid_hand)

        self.node.gamepad_callback(GamePad(button_a=True))

        self.assertFalse(self.node.solenoid_hand)

    def test_lb_starts_dribbling_sequence(self):
        self.node.gamepad_callback(GamePad(button_lb=True))

        self.assertTrue(self.node.dribbling)
        self.assertTrue(self.node.solenoid_push)
        self.assertTrue(self.node.solenoid_hand)

        for _ in range(self.node.PUSH_TICKS + 1):
            self.node.digital_and_analog_callback(DigitalAndAnalogFeedback(can_id=1))

        self.assertFalse(self.node.solenoid_push)
        self.assertTrue(self.node.solenoid_hand)

    def test_feedback_detection_closes_hand(self):
        self.node.gamepad_callback(GamePad(button_lb=True))

        for _ in range(self.node.HOLD_TICKS + 1):
            self.node.digital_and_analog_callback(DigitalAndAnalogFeedback(can_id=1))

        self.assertTrue(self.node.start_detect)

        self.node.digital_and_analog_callback(
            DigitalAndAnalogFeedback(
                can_id=self.node.DETECTION_CAN_ID,
                analog2_value=self.node.DETECTION_THRESHOLD + 0.1,
            )
        )

        self.assertFalse(self.node.dribbling)
        self.assertFalse(self.node.start_detect)
        self.assertFalse(self.node.solenoid_push)
        self.assertFalse(self.node.solenoid_hand)
        self.assertEqual(self.node.counter_loop, 0)


if __name__ == '__main__':
    unittest.main()
