# COMP0182 (Multi-Agent Systems): Lab Sheet 3

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1 (Mandatory): Install and connect the Remote PC (your Laptop) and Target Robot (TurtleBot3 Burger)
Follow the official tutorial from
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

**Important**: Make sure you select the correct version of ROS: **Noetic**

![TurtleBot3](/Week_03/img/noetic.png)

### Key points

**3. 1. PC Setup**

You can skip the introduction video first, it talks about setting up a TurtleBot3 in the Gazebo simulation world. You can go back and watch this after finishing this lab's work.

**1. 1. 1. Download and Install Ubuntu on PC**: 

You should have finished these steps in previous lab sessions. If yes, please go directly to the "1. 1. 3 Install Dependent ROS Packages" sub-section.
If not please do it.

**1. 1. 2. Install ROS on Remote PC**: 

You should have finished these steps in previous lab sessions. If yes, please go directly to the "1. 1. 3 Install Dependent ROS Packages" sub-section.
If not please do it.

**1. 1. 3 Install Dependent ROS Packages**

Just follow the instructions.

**1. 1. 4. Install TurtleBot3 Packages**

Just follow the instructions.

**1. 1. 5. Network Configuration**: 

Just follow the instructions.

**3. 2. SBC Setup**

**0. 2. 1. Prepare microSD Card and Reader**

Just follow the instructions.

**0. 2. 2. Download TurtleBot3 SBC Image**

Download the correct image file for your hardware and ROS version. Noetic version images are created based on Ubuntu 20.04. 

Please Download the "Raspberry Pi 4B (2GB or 4GB)" ROS Noetic image.

**0. 2. 3. Unzip the downloaded image file**

Just follow the instructions.

**0. 2. 4. Burn the image file**

Just follow the instructions.

**0.2.5. Resize the Partition**

[Optional] Be aware of selecting an incorrect disk or a partition. Partitioning a system disk of your PC may cause a serious system malfunction. You can skip this step. 

**0.2.6. Configure the WiFi Network Setting**

Just follow the instructions. If you are unable to go to this folder on your command prompt: "cd /media/$USER/writable/etc/netplan", you can go to this folder: "cd /etc/netplan".

**0.2.7. ROS Network Configuration**

Just follow the instructions.

**0.2.8. NEW LDS-02 Configuration**

**3. 3. OpenCR Setup**

Just follow the instructions.

**3. 4. Hardware Assembly**

This can be skipped as the turtlebots have already been built. 

**3. 5. Bringup**

Just follow the instructions.

**3. 6. Basic Operation**

Just follow the instructions.

Expected output:
Teleoperating of TurtleBot3 from your laptop.

Hints: 3 terminals from your PC, one for SSH, one for ROS (roscore), and one for teleoperation.

## Task 2: Finding single and multiple ArUco Markers using the camera
Follow [https://github.com/JacksonChiy/turtlebot3_burger_auto_navigation/blob/main/auto_aruco_marker_finder/launch/aruco_marker_finder.launch](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder)

## Task 3 (Optional): Find the ArUco Marker and navigate towards it. 

## Task 4 (Optional): Naïve obstacle avoidance using LiDAR
Follow https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/naive_obstacle_avoidance
