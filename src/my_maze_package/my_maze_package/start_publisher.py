import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class StartPublisher(Node):
    def __init__(self):
        super().__init__('start_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'start_topic', 10)
        self.timer = self.create_timer(2, self.publish_start)

    def publish_start(self):
        start_position = [0, 0]  # example start position (row 0, column 0)

        msg = Int32MultiArray()
        msg.data = start_position
        self.publisher_.publish(msg)
        self.get_logger().info('Published start position')

def main(args=None):
    rclpy.init(args=args)
    start_node = StartPublisher()
    rclpy.spin(start_node)
    start_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
