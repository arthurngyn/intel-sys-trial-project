from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_maze_package',
            executable='map_publisher',
            name='map_publisher',
            output='screen'
        ),
        Node(
            package='my_maze_package',
            executable='start_publisher',
            name='start_publisher',
            output='screen'
        ),
        Node(
            package='my_maze_package',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen'
        ),
        Node(
            package='my_maze_package',
            executable='path_publisher',
            name='path_publisher',
            output='screen'
        ),
    ])
