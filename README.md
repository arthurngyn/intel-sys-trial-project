# Pathfinding ROS2 Project

## Overview
This project implements a ROS2-based pathfinding system for grid-based maps. The application leverages several ROS2 nodes to manage map data, start and goal positions, and compute the shortest path between the start and goal using a basic A* algorithm.

## Features
- **Dynamic Pathfinding:** Computes a path in a grid-based map from a start position to a goal position.
- **ROS2 Integration:** Utilizes ROS2 topics and message types for communication between nodes.
- **Scalable Map Support:** Processes multi-dimensional grid data to support various map sizes.

## Nodes
### 1. **Pathfinding Node**
- **File:** `pathfinding_node.py`
- **Functionality:**
  - Subscribes to:
    - `map_topic`: Receives grid-based map data.
    - `start_topic`: Receives the start position.
    - `goal_topic`: Receives the goal position.
  - Publishes:
    - `path_topic`: Publishes the computed path as a sequence of coordinates.
  - **Pathfinding Algorithm:** Implements a basic A* algorithm to calculate the shortest path.

### 2. **Start Publisher Node**
- **File:** `start_publisher.py`
- **Functionality:**
  - Publishes the start position to the `start_topic` every 2 seconds.

### 3. **Map Publisher Node**
- **File:** `map_publisher.py`
- **Functionality:**
  - Publishes grid-based map data to the `map_topic` every second.

## ROS2 Topics
### Subscribed Topics
- `map_topic` (Type: `std_msgs/Int32MultiArray`): Grid-based map data where `0` represents open space and `1` represents walls.
- `start_topic` (Type: `std_msgs/Int32MultiArray`): Start position (e.g., `[0, 0]`).
- `goal_topic` (Type: `std_msgs/Int32MultiArray`): Goal position (e.g., `[5, 5]`).

### Published Topics
- `path_topic` (Type: `std_msgs/Int32MultiArray`): Computed path as a sequence of coordinates (e.g., `[0, 0, 0, 1, 0, 2, ...]`).

## Running the Project

### Prerequisites
- Install ROS2 (e.g., Humble, Foxy, or Galactic).
- Install Python dependencies for ROS2:
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions python3-rosdep
  ```
- Initialize ROS2 workspace:
  ```bash
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws
  colcon build
  source install/setup.bash
  ```


