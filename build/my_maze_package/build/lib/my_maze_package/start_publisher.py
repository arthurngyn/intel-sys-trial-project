# start_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StartPublisher(Node):
    def __init__(self):
        super().__init__('start_publisher')
        self.publisher_ = self.create_publisher(String, '/start', 10)
        self.timer = self.create_timer(1.0, self.publish_start)  # Publish every second
        self.get_logger().info('Start Publisher Node Started')

    def publish_start(self):
        start_position = '[0, 0]'  # Example starting position
        msg = String()
        msg.data = start_position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Start Position: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    start_publisher = StartPublisher()
    rclpy.spin(start_publisher)
    start_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
