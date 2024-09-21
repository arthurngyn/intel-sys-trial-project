from setuptools import find_packages, setup

package_name = 'my_maze_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthur@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        [
            # package name file name 
            'map_publisher = my_maze_package.map_publisher:main',
            'start_publisher = my_maze_package.start_publisher:main',
            'goal_publisher = my_maze_package.goal_publisher:main',
            'pathfinding_node = my_maze_package.pathfinding_node:main',
            'path_visualizer = my_maze_package.path_visualizer:main',
        ],
    },
)
