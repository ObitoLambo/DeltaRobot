import rclpy
from rclpy.node import Node
from custom_messages.msg import DigitalAndSolenoidCommand


class SolenoidControl(Node):
    def __init__(self):
        super().__init__('test_solenoid_node')
        self.gripper = True
        self.publish_period = 0.5

        self.solenoid_publisher = self.create_publisher(DigitalAndSolenoidCommand, '/publish_digital_solenoid', 10)
        self.publish_timer = self.create_timer(self.publish_period, self.publish_solenoid)

        self.publish_solenoid()
        self.get_logger().info(
            f'Publishing solenoid command on /publish_digital_solenoid every {self.publish_period} seconds'
        )

    def publish_solenoid(self):
        msg = DigitalAndSolenoidCommand()
        msg.can_id = 4
        msg.solenoid1_value = self.gripper
        self.solenoid_publisher.publish(msg)


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
