import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arthur/ros2_ws/install/my_maze_package/share/my_maze_package/install/my_maze_package'
