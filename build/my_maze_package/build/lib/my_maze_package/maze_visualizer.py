import os
import pygame
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Adjust this if using different message types
from nav_msgs.msg import OccupancyGrid  # For maze
from geometry_msgs.msg import PoseArray  # For path

# Maze visualization settings
CELL_SIZE = 20
MAZE_FILE = os.path.join(os.path.dirname(__file__), '/home/arthur/ros2_ws/src/my_maze_package/my_maze_package/maze.txt')

class MazeVisualizer(Node):
    def __init__(self):
        super().__init__('maze_visualizer')
        self.declare_parameter('maze_file', MAZE_FILE)

        # Initialize Pygame
        pygame.init()
        self.screen = None
        self.width = 800
        self.height = 600
        self.cell_size = CELL_SIZE
        self.setup_pygame()

        # Initialize Maze and Path
        self.maze = []
        self.path = []

        # ROS2 Subscriptions
        self.create_subscription(OccupancyGrid, 'maze', self.maze_callback, 10)
        self.create_subscription(PoseArray, 'path', self.path_callback, 10)

    def setup_pygame(self):
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Maze Visualizer')
        self.clock = pygame.time.Clock()

    def load_maze(self, filename):
        with open(filename, 'r') as file:
            return [list(map(int, line.split())) for line in file]

    def draw_maze(self):
        for y, row in enumerate(self.maze):
            for x, cell in enumerate(row):
                color = (0, 0, 0) if cell == 1 else (255, 255, 255)
                pygame.draw.rect(self.screen, color, pygame.Rect(x * self.cell_size, y * self.cell_size, self.cell_size, self.cell_size))

    def draw_path(self):
        for pose in self.path:
            pygame.draw.circle(self.screen, (255, 0, 0), (int(pose.x * self.cell_size), int(pose.y * self.cell_size)), 5)

    def maze_callback(self, msg):
        self.maze = [[cell for cell in row] for row in msg.data]  # Adjust based on the actual message format

    def path_callback(self, msg):
        self.path = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  # Process ROS2 callbacks
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    rclpy.shutdown()
                    sys.exit()

            self.screen.fill((200, 200, 200))
            self.draw_maze()
            self.draw_path()
            pygame.display.flip()
            self.clock.tick(30)  # Cap the frame rate at 30 FPS

def main(args=None):
    rclpy.init(args=args)
    visualizer = MazeVisualizer()
    visualizer.run()

if __name__ == '__main__':
    main()
