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

You should have finished these steps in previous lab sessions. If yes, skip this step and go to the "1. 1. 2 Install Dependent ROS Packages" sub-section.
If not please do it.

**1. 1. 2. Install ROS on Remote PC**: 

You should have finished these steps in previous lab sessions. If yes, skip this step and go to the "1. 1. 3 Install Dependent ROS Packages" sub-section.
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

[Optional] Be aware of selecting an incorrect disk or a partition, partitioning a system disk of your PC may cause a serious system malfunction. You can skip this step. 

**0.2.6. Configure the WiFi Network Setting**

Boot Up the Raspberry Pi
a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.
b. Connect input devices to the USB port of Raspberry Pi.
c. Insert the microSD card.
d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
e. Login with ID **ubuntu** and PASSWORD **turtlebot**.

Just follow the instructions. 

If you are unable to go to this folder on your command prompt: "cd /media/$USER/writable/etc/netplan", you can go to this folder: "cd /etc/netplan" and follow the instructions. 

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

In this task, we are gonna show you how to track ArUco marker and estimate their pose using USB camera in Python 3 on Ubuntu 20.04 on ROS Noetic.

**Installing the required package**:

So first let’s install the required packages:

Open a terminal on your Ubuntu 20.04. Type this command:
$ sudo apt-get install ros-noetic-usb-cam ros-noetic-aruco-ros

**Launch Files**:
In GitHub: https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch, there are three launch files,
01. usb_cam_stream_publisher.launch: USB image publisher
02. aruco_marker_finder.launch: single ArUco Marker finder
03. multiple_aruco_marker_finder.launch: multiple ArUco Markers finder

**Run ROS Master**:

Open first terminal, run this command to run the ROS master:

$ roscore

Let this terminal run the master always. Please do not close and terminate this window. 

**Build or compile your workspace**:

Open second terminal, run this command to compile your workspace:

$ cd catkin_ws

$ catkin_ws$ catkin_make

**Publishing Images from Camera and Estimating the Pose**:

Open third terminal, go to this folder:

$ catkin_ws$ cd catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/

and run this command to publish USB camera images and estimate the pose of the ArUco Marker:

$ catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch$ roslaunch usb_cam_stream_publisher.launch

**Finding the ArUco Marker**:

Open fourth terminal, go to this folder:

$ catkin_ws$ cd catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/

and run this command to find the ArUco Marker:

$ catkin_ws$ cd catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch$ roslaunch aruco_marker_finder.launch markerId:=701 markerSize:=0.05

**Open Rqt GUI**

Open fifth terminal, run this command to see the results:

$ rosrun rqt_gui rqt_gui

**Pose of ArUco Marker**

Open sixth terminal, run this command to see the pose of the marker:

$ rostopic echo /aruco_single/pose

Now, take the camera and keep on the ArUco Marker Id = 701 image, you can see the results rqt_gui window.

**Reference**:

GitHub: [Week_03/turtlebot3_burger_auto_navigation/](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder)

## Task 3 (Optional): Find the ArUco Marker and navigate towards it. 

## Task 4 (Optional): Naïve obstacle avoidance using LiDAR
Follow https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/naive_obstacle_avoidance
