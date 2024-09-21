# map_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(String, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)  # Publish every second
        self.get_logger().info('Map Publisher Node Started')

    def publish_map(self):
        # Example 5x5 grid map as a string (where '0' is an open path and '1' is a wall)
        map_data = '[[0, 1, 0, 0, 0], [0, 1, 1, 1, 0], [0, 0, 0, 1, 0], [0, 1, 0, 0, 0], [0, 0, 0, 1, 0]]'
        msg = String()
        msg.data = map_data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Map: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
