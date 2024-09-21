import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import heapq

class PathfindingNode(Node):
    def __init__(self):
        super().__init__('pathfinding_node')
        # subscribe to the topics to get he information 
        self.map_sub = self.create_subscription(
            Int32MultiArray, 'map_topic', self.map_callback, 10)
        self.start_sub = self.create_subscription(
            Int32MultiArray, 'start_topic', self.start_callback, 10)
        self.goal_sub = self.create_subscription(
            Int32MultiArray, 'goal_topic', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Int32MultiArray, 'path_topic', 10)

        #initialize local variables 
        self.map_data = None
        self.start_pos = None
        self.goal_pos = None

    def map_callback(self, msg):
        self.map_data = [msg.data[i:i + 6] for i in range(0, len(msg.data), 6)]
        self.get_logger().info('Received map data')
        self.try_pathfinding()

    def start_callback(self, msg):
        self.start_pos = msg.data
        self.get_logger().info(f'Received start position: {self.start_pos}')
        self.try_pathfinding()

    def goal_callback(self, msg):
        self.goal_pos = msg.data
        self.get_logger().info(f'Received goal position: {self.goal_pos}')
        self.try_pathfinding()

    def try_pathfinding(self):
        if (self.map_data is  not None and self.start_pos is not None and 
            self.goal_pos is not None ):
            try:
                path = self.find_path(self.map_data, tuple(self.start_pos),  tuple(self.goal_pos))
                if path:
                    path_msg =  Int32MultiArray()

                    path_msg.data = [coord for point in path for coord in  point]  # flatten the list of tuples
                    self.path_pub.publish(path_msg )        
                    self.get_logger().info('Published path')
                else:
                    self.get_logger().info('No path found')

            except Exception as e:
                self.get_logger().error(f'Error in pathfinding: {str(e)}')
        else:
            self.get_logger().info('Waiting for map, start, and goal data')

    def find_path(self, maze, start, goal):
        rows, cols = len(maze), len(maze[0])
        heap = [(0, start)]
        came_from = { start: None }
        cost_so_far = {start: 0}

        while heap:
            current_cost, current = heapq.heappop(heap)

            if current == goal:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                next_pos = (current[0] + dx, current[1] + dy)
                if (0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and 
                    maze[next_pos[0]][next_pos[1]] == 0):
                    new_cost = cost_so_far[current] + 1
                    if (next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]):
                        cost_so_far[next_pos] = new_cost
                        priority = new_cost
                        heapq.heappush(heap, (priority, next_pos))
                        came_from[next_pos] = current

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from.get(current, start)
        path.append(start)
        path.reverse()
        return path

def main(args=None):
    rclpy.init(args=args)
    pathfinding_node = PathfindingNode()
    rclpy.spin(pathfinding_node)
    pathfinding_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
