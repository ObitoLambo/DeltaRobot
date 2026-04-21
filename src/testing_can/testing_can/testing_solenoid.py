import rclpy
from rclpy.node import Node
from custom_messages.msg import DigitalAndSolenoidCommand


class SolenoidControl(Node):
    def __init__(self):
        super().__init__('test_solenoid_node')
        self.gripper_open = True
        self.gripper_close = False
        self.publish_period = 1
        self.close_delay = 0.5  # seconds before closing

        self.solenoid_publisher = self.create_publisher(DigitalAndSolenoidCommand, '/publish_digital_solenoid', 10)
        self.publish_timer = self.create_timer(self.publish_period, self.open_gripper)
        self.close_timer = None  # one-shot timer for closing

        self.open_gripper()
        self.get_logger().info(
            f'Publishing solenoid command on /publish_digital_solenoid every {self.publish_period} seconds'
        )

    def open_gripper(self):
        msg = DigitalAndSolenoidCommand()
        msg.can_id = 4
        msg.solenoid1_value = self.gripper_open
        self.solenoid_publisher.publish(msg)
        self.get_logger().info('Gripper OPENED')

        # Schedule close after delay (one-shot timer)
        if self.close_timer:
            self.close_timer.cancel()
        self.close_timer = self.create_timer(self.close_delay, self.close_gripper)

    def close_gripper(self):
        # Cancel so it only fires once
        self.close_timer.cancel()

        msg = DigitalAndSolenoidCommand()
        msg.can_id = 4
        msg.solenoid1_value = self.gripper_close
        self.solenoid_publisher.publish(msg)
        self.get_logger().info('Gripper CLOSED')


def main(args=None):
    rclpy.init(args=args)
    node = SolenoidControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()