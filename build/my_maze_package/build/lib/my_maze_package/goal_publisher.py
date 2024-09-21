# goal_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(String, '/goal', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)  # Publish every second
        self.get_logger().info('Goal Publisher Node Started')

    def publish_goal(self):
        goal_position = '[4, 4]'  # Example goal position
        msg = String()
        msg.data = goal_position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Goal Position: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
