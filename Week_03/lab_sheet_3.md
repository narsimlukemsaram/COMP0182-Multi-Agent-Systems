# COMP0182 (Multi-Agent Systems): Lab Sheet 3

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1 (Mandatory): Install and connect the Remote PC (your Laptop) and Target Robot (TurtleBot3 Burger)
Follow the official tutorial from
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Make sure you select the correct version of ROS: **Noetic**
![TurtleBot3](/Week_03/img/noetic.png)

### Key points

**3. 1. PC Setup**

You can skip the introduction video first, it is talking about setting up a TurtleBot3 in the Gazebo simulation world. You can go back and watch this after finishing this lab's work.

**1. 1. 1. Download and Install Ubuntu on PC  & 1. 1. 2. Install ROS on Remote PC**: 

You should have finished these steps in previous lab sessions, if not please do it.

Please follow the instructions in 
**1. 1. 3 Install Dependent ROS Packages and 1. 1. 4. Install TurtleBot3 Packages**

**1. 1. 5. Network Configuration**: 

- `ROS_MASTER_URI` specifies the location of your ROS master which means  all other machines will communicate with it. This can be either your PC or the Raspberry pi, but you have to make it consistent among all the machines.
- `ROS_HOSTNAME` Specifies the hostname or IP address of your machine, so it is unique for each machine.

**3.2. SBC Setup**

Just follow the instructions.

**3.3 OpenCR Setup**

Just follow the instructions.

**3.4 Hardware Assembly**

This can be skipped as the turtlebots have already been built.

**3.5 Bringup**

Just follow the instructions.

**3.6 Basic Operation**

Just follow the instructions.

Expected output:
Teleoperating of TurtleBot3 from your laptop.

Hints: 3 terminals from your pc, one for SSH, one for roscore, and one for teleoperation.

## Task 2: Finding Single and Multiple ArUco Markers using Camera
Follow [https://github.com/JacksonChiy/turtlebot3_burger_auto_navigation/blob/main/auto_aruco_marker_finder/launch/aruco_marker_finder.launch](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder)

## Task 3 (Optional): Find the ArUco Marker and navigate towards it. 

## Task 4 (Optional): Naïve obstacle Avoidance using LiDAR
Follow https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/naive_obstacle_avoidance
