import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        # creating a publisher object 
        # ( ROS message type, ROS2 topic name, Queue size 10 is a good moderate number that can still publish relevant data)
        self.map_pub = self.create_publisher(Int32MultiArray, 'map_topic', 10) 
        
        self.timer = self.create_timer(1.0, self.publish_map_data)  # Publish every 1 second

    def publish_map_data(self):
        # create a multi dimesnional array of 32-bit integers 
        # good for sending grid data such as a map 
        msg = Int32MultiArray()
        # 0 is open room and 1 is a wall 
        msg.data = [0, 1, 1, 0, 0, 0,
                    0, 1, 0, 1, 0, 0,
                    0, 1, 1, 0, 0, 0,
                    0, 1, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 1, 0, 1, 0, 0]
        self.map_pub.publish(msg)
        # publish the map data and log it 
        self.get_logger().info('Published map data')

def main(args=None):
    #initialize ros2 python client libary 
    rclpy.init(args=args)
    #map publisher inherits the Node class 
    publisher = MapPublisher()
    #loops the node 
    rclpy.spin(publisher)
    #cleans up resources from node after stopping the node 
    publisher.destroy_node()
    #cleans up resrouces from rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()
