#!/usr/bin/env python3
"""
ROS Node for Autonomous Navigation of a Single TurtleBot Using ArUco Markers

This script navigates a TurtleBot along a predefined path by reading waypoints from a YAML file.
It uses ArUco markers for localization and computes the required transformations to map simulation
coordinates to real-world coordinates.

Date: 20 November 2024

Requirements:
- ROS (Robot Operating System)
- OpenCV with ArUco module
- PyYAML
"""

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import yaml
import numpy as np
import cv2

def convert_sim_to_real_pose(x, y, matrix):
    """
    Converts simulation coordinates to real-world coordinates using a perspective transformation matrix.

    Parameters:
    - x (float): X-coordinate in simulation.
    - y (float): Y-coordinate in simulation.
    - matrix (np.ndarray): 3x3 perspective transformation matrix.

    Returns:
    - Tuple[float, float]: Transformed X and Y coordinates in real-world.
    """
    # Create a homogeneous coordinate for the point
    point = np.array([x, y, 1])

    # Apply the perspective transformation
    transformed_point = np.dot(matrix, point)

    # Normalize to get the actual coordinates
    transformed_point = transformed_point / transformed_point[2]

    return transformed_point[0], transformed_point[1]

def check_goal_reached(current_pose, goal_x, goal_y, tolerance):
    """
    Checks if the robot has reached the goal position within a specified tolerance.

    Parameters:
    - current_pose (PoseStamped): Current pose of the robot.
    - goal_x (float): Goal X-coordinate.
    - goal_y (float): Goal Y-coordinate.
    - tolerance (float): Acceptable distance from the goal to consider it reached.

    Returns:
    - bool: True if goal is reached, False otherwise.
    """
    # Get current position
    current_x = current_pose.pose.position.x
    current_y = current_pose.pose.position.y

    # Check if within tolerance
    if (abs(current_x - goal_x) <= tolerance and abs(current_y - goal_y) <= tolerance):
        return True
    else:
        return False

def navigation(turtlebot_name, aruco_id, goal_list):
    """
    Navigates the TurtleBot through a list of waypoints.

    Parameters:
    - turtlebot_name (str): Name of the TurtleBot.
    - aruco_id (str): ArUco marker ID used for localization.
    - goal_list (List[Tuple[float, float]]): List of (X, Y) coordinates as waypoints.
    """
    current_position_idx = 0  # Index of the current waypoint

    # Publisher to send velocity commands to the robot
    cmd_pub = rospy.Publisher(f'/{turtlebot_name}/cmd_vel', Twist, queue_size=1)

    # Wait for the initial pose message from the ArUco marker
    init_pose = rospy.wait_for_message(f'/{aruco_id}/aruco_single/pose', PoseStamped)

    # Initialize Twist message for velocity commands
    twist = Twist()

    # Loop until all waypoints are reached or ROS is shut down
    while current_position_idx < len(goal_list) and not rospy.is_shutdown():
        # Get current goal coordinates
        goal_x, goal_y = goal_list[current_position_idx]

        # Check if the goal has been reached
        if check_goal_reached(init_pose, goal_x, goal_y, tolerance=0.1):
            rospy.loginfo(f"Waypoint {current_position_idx + 1} reached: Moving to next waypoint.")
            current_position_idx += 1  # Move to the next waypoint

            # If all waypoints are reached, exit the loop
            if current_position_idx >= len(goal_list):
                rospy.loginfo("All waypoints have been reached.")
                break

        # Update the current pose
        init_pose = rospy.wait_for_message(f'/{aruco_id}/aruco_single/pose', PoseStamped)

        # Extract the current orientation in radians from quaternion
        orientation_q = init_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        current_orientation = yaw  # Current heading of the robot

        # Calculate the difference between the goal and current position
        dx = goal_x - init_pose.pose.position.x
        dy = goal_y - init_pose.pose.position.y
        distance = math.hypot(dx, dy)  # Euclidean distance to the goal
        goal_direction = math.atan2(dy, dx)  # Angle to the goal

        # Normalize angles to range [0, 2π)
        current_orientation = (current_orientation + 2 * math.pi) % (2 * math.pi)
        goal_direction = (goal_direction + 2 * math.pi) % (2 * math.pi)

        # Compute the smallest angle difference
        theta = goal_direction - current_orientation

        # Adjust theta to be within [-π, π]
        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi

        # Log debug information
        rospy.logdebug(f"Current Position: ({init_pose.pose.position.x:.2f}, {init_pose.pose.position.y:.2f})")
        rospy.logdebug(f"Goal Position: ({goal_x:.2f}, {goal_y:.2f})")
        rospy.logdebug(f"Current Orientation: {current_orientation:.2f} rad")
        rospy.logdebug(f"Goal Direction: {goal_direction:.2f} rad")
        rospy.logdebug(f"Theta (Angle to Goal): {theta:.2f} rad")
        rospy.logdebug(f"Distance to Goal: {distance:.2f} meters")

        # Control parameters (adjust these as needed)
        k_linear = 0.5    # Linear speed gain
        k_angular = 2.0   # Angular speed gain

        # Compute control commands
        linear_velocity = k_linear * distance * math.cos(theta)  # Move forward towards the goal
        angular_velocity = -k_angular * theta  # Rotate towards the goal direction

        # Limit maximum speeds if necessary
        max_linear_speed = 0.2  # meters per second
        max_angular_speed = 1.0  # radians per second

        linear_velocity = max(-max_linear_speed, min(max_linear_speed, linear_velocity))
        angular_velocity = max(-max_angular_speed, min(max_angular_speed, angular_velocity))

        # Set Twist message
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        # Publish the velocity commands
        cmd_pub.publish(twist)

        # Sleep to maintain the loop rate
        rospy.sleep(0.1)  # Adjust the sleep duration as needed

def get_transformation_matrix(aruco_markers):
    """
    Detects corner ArUco markers and calculates the perspective transformation matrix.

    Parameters:
    - aruco_markers (List[str]): List of ArUco marker IDs used for the transformation.

    Returns:
    - np.ndarray: 3x3 perspective transformation matrix.
    """
    # Dictionary to store the poses of the ArUco markers
    marker_poses = {}

    # Wait for ArUco marker poses to define transformation between simulation and real-world coordinates
    for marker_id in aruco_markers:
        try:
            # Wait for the pose of each ArUco marker
            pose = rospy.wait_for_message(f'/{marker_id}/aruco_single/pose', PoseStamped, timeout=5)
            marker_poses[marker_id] = (pose.pose.position.x, pose.pose.position.y)
            rospy.loginfo(f"Received pose for marker {marker_id}: x={pose.pose.position.x}, y={pose.pose.position.y}")
        except rospy.ROSException:
            rospy.logerr(f"Timeout while waiting for pose of marker {marker_id}")
            raise

    # Define real-world and simulation points for the perspective transformation
    real_points = np.float32([
        marker_poses['id503'],  # Bottom-left corner in real world
        marker_poses['id502'],  # Bottom-right corner in real world
        marker_poses['id500'],  # Top-left corner in real world
        marker_poses['id501']   # Top-right corner in real world
    ])

    sim_points = np.float32([
        [0, 0],     # Bottom-left corner in simulation
        [10, 0],    # Bottom-right corner in simulation
        [0, 10],    # Top-left corner in simulation
        [10, 10]    # Top-right corner in simulation
    ])

    # Calculate the perspective transformation matrix
    matrix = cv2.getPerspectiveTransform(sim_points, real_points)

    rospy.loginfo("Perspective transformation matrix calculated successfully.")

    return matrix

def read_and_transform_waypoints(file_path, matrix):
    """
    Reads waypoints from a YAML file and transforms them from simulation to real-world coordinates.

    Parameters:
    - file_path (str): Path to the YAML file containing the schedule.
    - matrix (np.ndarray): Perspective transformation matrix.

    Returns:
    - List[Tuple[float, float]]: List of transformed waypoints.
    """
    # Read the schedule from the YAML file
    def read_yaml_file(file_path):
        """
        Reads the schedule from a YAML file.

        Parameters:
        - file_path (str): Path to the YAML file.

        Returns:
        - dict: Dictionary containing the schedule data.
        """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['schedule']  # Returns a dictionary of steps

    try:
        # Load schedule data from YAML file
        schedule_data = read_yaml_file(file_path)
    except Exception as e:
        rospy.logerr(f"Failed to read schedule YAML file: {e}")
        raise

    coordinates = []  # List to store transformed waypoints

    # Process waypoints for each agent
    for agent_id, steps in schedule_data.items():
        rospy.loginfo(f"Processing agent {agent_id}")

        for step in steps:
            # Simulation coordinates
            sim_x = step['x']
            sim_y = step['y']

            # Transform simulation coordinates to real-world coordinates
            real_x, real_y = convert_sim_to_real_pose(sim_x, sim_y, matrix)

            rospy.loginfo(f"Transformed simulation coordinates ({sim_x}, {sim_y}) to real-world coordinates ({real_x:.2f}, {real_y:.2f})")

            # Append the transformed coordinates to the list
            coordinates.append((real_x, real_y))

        break  # Remove this if you want to process multiple agents

    return coordinates

def main():
    """
    Main function to initialize the ROS node and start the navigation process.
    """
    rospy.init_node('goal_pose')

    # List of ArUco marker IDs used for the transformation
    aruco_markers = ['id500', 'id501', 'id502', 'id503']

    try:
        # Get the transformation matrix using the corner detection function
        matrix = get_transformation_matrix(aruco_markers)
    except Exception as e:
        rospy.logerr(f"Failed to get transformation matrix: {e}")
        return

    try:
        # Read and transform waypoints from the YAML file
        coordinates = read_and_transform_waypoints("./cbs_output.yaml", matrix)
    except Exception as e:
        rospy.logerr(f"Failed to read and transform waypoints: {e}")
        return

    # Start navigation with the first agent's waypoints
    turtlebot_name = "turtle1"  # Name of your TurtleBot
    aruco_id = "id402"          # ArUco marker ID for localization

    # Begin the navigation process
    navigation(turtlebot_name, aruco_id, coordinates)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
