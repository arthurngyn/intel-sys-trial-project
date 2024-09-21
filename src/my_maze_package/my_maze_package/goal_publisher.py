import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

#inherit node class 
class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'goal_topic', 10)
        self.timer = self.create_timer(2, self.publish_goal)

    def publish_goal(self):
        goal_position = [2, 4]  # example goal position (row 5, column 5)

        msg = Int32MultiArray()
        msg.data = goal_position
        self.publisher_.publish(msg)
        self.get_logger().info('Published goal position')

def main(args=None):
    rclpy.init(args=args)
    goal_node = GoalPublisher()
    rclpy.spin(goal_node)
    goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
