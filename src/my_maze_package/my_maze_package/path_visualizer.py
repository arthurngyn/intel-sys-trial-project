import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import pygame
import time

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.map_data = None
        self.path = set()  # use a set to store path coordinates
        self.start = None  # to store start point
        self.goal = None  # to store goal point

        # subscriptions
        self.create_subscription(
            Int32MultiArray,
            'map_topic',
            self.map_callback,
            10
        )
        self.create_subscription(
            Int32MultiArray,
            'path_topic',
            self.path_callback,
            10
        )
        self.create_subscription(
            Int32MultiArray,
            'start_topic',
            self.start_callback,
            10
        )
        self.create_subscription(
            Int32MultiArray,
            'goal_topic',
            self.goal_callback,
            10
        )

        # initialize pygame
        pygame.init()
        self.map_width = 6
        self.map_height = 6
        self.cell_size = 50
        self.screen = pygame.display.set_mode((self.map_width * self.cell_size, self.map_height * self.cell_size))
        pygame.display.set_caption('Path Visualizer')
        self.clock = pygame.time.Clock()

    def map_callback(self, msg):
        self.map_data = [msg.data[i:i + 6] for i in range(0, len(msg.data), 6)]
        self.get_logger().info(f'Received map data: {self.map_data}')
        if self.path and self.start and self.goal:
            self.visualize_map()

    def path_callback(self, msg):
        # convert the flattened list into a list of tuples
        self.path = set((msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2))
        self.get_logger().info(f'Received path data: {self.path}')
        if self.map_data and self.start and self.goal:
            self.visualize_map()

    def start_callback(self, msg):
        self.start = (msg.data[0], msg.data[1])  # start point (x, y)
        self.get_logger().info(f'Received start point: {self.start}')
        if self.map_data and self.path and self.goal:
            self.visualize_map()

    def goal_callback(self, msg):
        self.goal = (msg.data[0], msg.data[1])  # goal point (x, y)
        self.get_logger().info(f'Received goal point: {self.goal}')
        if self.map_data and self.path and self.start:
            self.visualize_map()

    def visualize_map(self):
        if self.map_data is None:
            return

        # clear the screen
        self.screen.fill((0, 0, 0))

        # draw the map
        for x in range(self.map_height): #start at the top row and go down 
            for y in range(self.map_width): #go from left to right 
                cell = self.map_data[x][y]
                color = (255, 255, 255) if cell == 0 else (0, 0, 0)  # white for empty black for obstacle
                pygame.draw.rect(
                    self.screen,
                    color,
                    (y * self.cell_size, x * self.cell_size, self.cell_size, self.cell_size)
                )

        # draw the path
        for (x, y) in self.path: 
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                pygame.draw.rect(
                    self.screen,
                    (0, 0, 255),  # blue for path
                    (y * self.cell_size, x * self.cell_size, self.cell_size, self.cell_size)
                )
            else:
                self.get_logger().warn(f'Path coordinate out of bounds: ({x}, {y})')

        # draw the start point
        if self.start:
            pygame.draw.rect(
                self.screen,
                (0, 255, 0),  # green for start point
                (self.start[1] * self.cell_size, self.start[0] * self.cell_size, self.cell_size, self.cell_size)
            )

        # draw the goal point
        if self.goal:
            pygame.draw.rect(
                self.screen,
                (255, 0, 0),  # red for goal point
                (self.goal[1] * self.cell_size, self.goal[0] * self.cell_size, self.cell_size, self.cell_size)
            )

        # update the display
        pygame.display.flip()

        # handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                return

        # wait to let the user see the final result
        time.sleep(3)  

        

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
