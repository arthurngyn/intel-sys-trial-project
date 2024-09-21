# File: src/my_maze_package/path_planner.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import heapq

class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')
        self.map_subscription = self.create_subscription(String, 'map', self.map_callback, 10)
        self.start_subscription = self.create_subscription(String, 'start', self.start_callback, 10)
        self.goal_subscription = self.create_subscription(String, 'goal', self.goal_callback, 10)
        self.path_publisher = self.create_publisher(String, 'path', 10)

        self.map = None
        self.start = None
        self.goal = None

    def map_callback(self, msg):
        self.map = np.array(eval(msg.data))  # Convert the string into a NumPy array
        self.calculate_path_if_ready()

    def start_callback(self, msg):
        self.start = tuple(eval(msg.data))  # Convert string into tuple for start coordinates
        self.calculate_path_if_ready()

    def goal_callback(self, msg):
        self.goal = tuple(eval(msg.data))  # Convert string into tuple for goal coordinates
        self.calculate_path_if_ready()

    def calculate_path_if_ready(self):
        if self.map is not None and self.start is not None and self.goal is not None:
            path = self.a_star(self.map, self.start, self.goal)
            if path:
                path_str = str(path)
                self.path_publisher.publish(String(data=path_str))
                self.get_logger().info(f'Path: {path_str}')
            else:
                self.get_logger().warn('No path found.')

    def a_star(self, grid, start, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data[::-1]

            close_set.add(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + 1

                if 0 <= neighbor[0] < grid.shape[0]:
                    if 0 <= neighbor[1] < grid.shape[1]:
                        if grid[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        continue
                else:
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

        return False

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
