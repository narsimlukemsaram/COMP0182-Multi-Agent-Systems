# Autonomous Navigation of TurtleBot Using ROS and ArUco Markers

This document provides an overview of the Python script for autonomous navigation of a TurtleBot using ROS (Robot Operating System) and ArUco markers. It explains how the code works and how to run it.

## Table of Contents

- [Introduction](#introduction)
- [Code Overview](#code-overview)
  - [Main Components](#main-components)
  - [Function Explanations](#function-explanations)
- [YAML Schedule File](#yaml-schedule-file)
  - [Structure of `cbs_output.yaml`](#structure-of-cbs_outputyaml)
- [Running the Code](#running-the-code)
  - [Adjusting Parameters](#adjusting-parameters)
  - [Setting Up Namespaces for TurtleBot](#setting-up-namespaces-for-turtlebot)
  - [Executing the Script](#executing-the-script)
- [Hints for Multi-agent Auto Navigation](#hints-for-multi-agent-auto-navigation)

## Introduction

This script enables a TurtleBot to navigate autonomously along a predefined path using ROS and ArUco markers for localization. The waypoints for the path are read from a YAML file (`cbs_output.yaml`). The script performs the following tasks:

1. **Corner Detection**: Uses ArUco markers placed at known positions to define a mapping between simulation and real-world coordinates.
2. **Coordinate Transformation**: Computes a perspective transformation matrix to convert simulation waypoints to real-world coordinates.
3. **Navigation**: Guides the TurtleBot through the list of transformed waypoints using proportional control.

## Code Overview

### Main Components

The script consists of several key components:

1. **Import Statements**: Import necessary libraries and modules.
2. **Function Definitions**:
   - `get_transformation_matrix()`: Calculates the perspective transformation matrix using ArUco markers.
   - `read_and_transform_waypoints()`: Reads waypoints from the YAML file and transforms them.
   - `convert_sim_to_real_pose()`: Converts simulation coordinates to real-world coordinates.
   - `navigation()`: Controls the robot to navigate through the waypoints.
   - `check_goal_reached()`: Checks if the robot has reached the current waypoint.
3. **Main Function**: Orchestrates the execution by calling the necessary functions.
4. **ROS Node Initialization**: Initializes the ROS node and handles exceptions.

### Function Explanations

#### 1. `get_transformation_matrix(aruco_markers)`

- **Purpose**: Calculates the perspective transformation matrix using ArUco markers placed at the corners of the environment.
- **Parameters**:
  - `aruco_markers` (list): List of ArUco marker IDs (e.g., `['id500', 'id501', 'id502', 'id503']`).
- **Returns**: 3x3 perspective transformation matrix.

#### 2. `read_and_transform_waypoints(file_path, matrix)`

- **Purpose**: Reads waypoints from a YAML file and transforms them into real-world coordinates.
- **Parameters**:
  - `file_path` (str): Path to the YAML file (`"cbs_output.yaml"`).
  - `matrix` (numpy.ndarray): Perspective transformation matrix.
- **Returns**: List of transformed waypoints as `(x, y)` tuples.

#### 3. `convert_sim_to_real_pose(x, y, matrix)`

- **Purpose**: Transforms simulation coordinates `(x, y)` to real-world coordinates using the provided transformation matrix.
- **Parameters**:
  - `x` (float): X-coordinate in simulation.
  - `y` (float): Y-coordinate in simulation.
  - `matrix` (numpy.ndarray): 3x3 perspective transformation matrix.
- **Returns**: Tuple `(real_x, real_y)` representing real-world coordinates.

#### 4. `navigation(turtlebot_name, aruco_id, goal_list)`

- **Purpose**: Guides the TurtleBot through the list of waypoints.
- **Parameters**:
  - `turtlebot_name` (str): Name of the TurtleBot (e.g., `"turtle1"`).
  - `aruco_id` (str): ArUco marker ID used for localization.
  - `goal_list` (list): List of waypoints as `(x, y)` tuples.

#### 5. `check_goal_reached(current_pose, goal_x, goal_y, tolerance)`

- **Purpose**: Determines if the robot is within a specified tolerance of the goal position.
- **Parameters**:
  - `current_pose` (`PoseStamped`): Current position of the robot.
  - `goal_x` (float): X-coordinate of the goal.
  - `goal_y` (float): Y-coordinate of the goal.
  - `tolerance` (float): Acceptable distance from the goal.
- **Returns**: `True` if goal is reached, `False` otherwise.

## YAML Schedule File

### Structure of `cbs_output.yaml`

The YAML file contains the schedule of waypoints for the TurtleBot. It should have the following structure:

```yaml
schedule:
  1:
    - t: 0
      x: 9
      y: 9
    - t: 1
      x: 8
      y: 9
      # More waypoints as needed
  # More agents if necessary
```

Note: In the current script, only the first agent's waypoints are processed due to the `break` statement in the read_and_transform_waypoints() function. Remove the `break` if you wish to process multiple agents.

## Running the Code

### Adjusting Parameters

Before running the script, you may need to adjust some parameters to match your setup:

- **ArUco Marker IDs**: Ensure the `aruco_markers` list in `get_transformation_matrix()` matches the IDs of the markers placed at the corners of your environment.
- **TurtleBot Name**: Set the `turtlebot_name` variable in the `main()` function to the correct name of your TurtleBot.
- **Localization ArUco ID**: Set the `aruco_id` variable in the `main()` function to the ID of the ArUco marker used for the robot's localization.
- **Control Gains**: Adjust `k_linear` and `k_angular` in the `navigation()` function if necessary to suit your robot's dynamics.

### Setting Up Namespaces for TurtleBot

> **This is essential to understand before experimenting with multiple agents.**

When using a custom `turtlebot_name` and including it in topic names (e.g., `/tb3_0/cmd_vel`), you need to configure the TurtleBot's onboard computer (e.g., the TurtleBot Pi) to subscribe to the correct namespaced topics.

#### Launching TurtleBot Bringup with Namespace

---

Use the following command to launch the TurtleBot bringup with a specific namespace:

```bash
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_0" set_lidar_frame_id:="tb3_0/base_scan"
```

Explanation:

- `ROS_NAMESPACE=tb3_0`: Sets the namespace to `tb3_0`.
- `roslaunch` turtlebot3_bringup turtlebot3_robot.launch: Launches the TurtleBot3 bringup launch file.
- `multi_robot_name:="tb3_0"`: Sets the robot name parameter to `tb3_0`.
- `set_lidar_frame_id:="tb3_0/base_scan"`: Sets the frame ID for the LiDAR sensor (optional).

#### Adjusting Launch Files

---

If you prefer to modify the launch files (`turtlebot3_bringup/launch/turtlebot3_robot.launch`) directly, set the arguments accordingly.

#### Consistency with the Python Script

---

Ensure that the `turtlebot_name` used in your Python script matches the namespace set when launching the TurtleBot bringup.

- In the Python script:

```python
turtlebot_name = "tb3_0"  # Name of your TurtleBot (namespace)
aruco_id = "id100"        # ArUco marker ID for localization

# Begin the navigation process
navigation(turtlebot_name, aruco_id, coordinates)
```

- In the Launch command:

```bash
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_0" set_lidar_frame_id:="tb3_0/base_scan"
```

### Executing the Script

1. Making the Script Executable

```bash
chmod +x your_script_name.py
```

2. Run the Script Using ROS

```bash
rosrun auto_navigation your_script_name.py
```

Replace `your_script_name.py` with the name of your script.

3. Monitor the Output

- Observe the console for logs indicating the progress of the robot.
- Ensure that ArUco markers are being detected and that the transformation matrix is calculated successfully.
- Check that the robot is moving towards the waypoints.

## Hints for Multi-agent Auto Navigation

Coming soon!

> This code and documentation should suffice to start your experiment with multiple agents. Take a look at the `final_challenge.py` implementation in the PyBullet simulation code as reference.
