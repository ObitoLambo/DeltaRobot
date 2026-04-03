import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SimulinkSubscriber(Node):
    def __init__(self):
        super().__init__('simulink_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            '/simulink_topic',   # must match Simulink Publish block topic
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimulinkSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()