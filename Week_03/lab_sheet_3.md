# COMP0182 (Multi-Agent Systems): Lab Sheet 3

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1 (Mandatory): Install and connect the Remote PC (your Laptop) and Target Robot (TurtleBot3 Burger)
Follow **Chatper 3: Quick Start Guide** from official tutorial from
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

**Important**: Make sure you select the correct version of ROS: **Noetic**

![TurtleBot3](/Week_03/img/noetic.png)

### Key points 
These are the points that you need to pay attention to while following the offcial tutorial. For the steps that are not mentioned here, just follow the content from the tutorial.

**3. 1. PC Setup**

You can skip the introduction video first, it talks about setting up a TurtleBot3 in the Gazebo simulation world. You can go back and watch this after finishing this lab's work.

**1. 1. 1. Download and Install Ubuntu on PC**: 

You should have finished these steps in previous lab sessions. If yes, skip this step and go to the "1. 1. 2 Install Dependent ROS Packages" sub-section.
If not please do it.

**1. 1. 2. Install ROS on Remote PC**: 

You should have finished these steps in previous lab sessions. If yes, skip this step and go to the "1. 1. 3 Install Dependent ROS Packages" sub-section.
If not please do it.

<!-- **1. 1. 3 Install Dependent ROS Packages**

Just follow the instructions.

**1. 1. 4. Install TurtleBot3 Packages**

Just follow the instructions.

**1. 1. 5. Network Configuration**: 

Just follow the instructions. -->

**3. 2. SBC Setup**

<!-- **0. 2. 1. Prepare microSD Card and Reader**

Just follow the instructions. -->

**0. 2. 2. Download TurtleBot3 SBC Image**

Download the correct image file for your hardware and ROS version. Noetic version images are created based on Ubuntu 20.04. 

Please Download the "Raspberry Pi 4B (2GB or 4GB)" ROS Noetic image.

<!-- **0. 2. 3. Unzip the downloaded image file**

Just follow the instructions.

**0. 2. 4. Burn the image file**

Just follow the instructions. -->

**0.2.5. Resize the Partition**

[Optional] Be aware of selecting an incorrect disk or a partition, partitioning a system disk of your PC may cause a serious system malfunction. You can skip this step. 

**0.2.6. Configure the WiFi Network Setting**

Boot Up the Raspberry Pi:

- Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.

- Connect input devices to the USB port of Raspberry Pi.

- Insert the microSD card.

- Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.

- Login with ID **ubuntu** and PASSWORD **turtlebot**.

Just follow the instructions. 

If you are unable to go to this folder on your command prompt: "cd /media/$USER/writable/etc/netplan", you can go to this folder: "cd /etc/netplan" and follow the instructions. 

<!-- **0.2.7. ROS Network Configuration**

Just follow the instructions.

**0.2.8. NEW LDS-02 Configuration** -->

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

In this task, we are gonna show you how to track ArUco markers and estimate their pose using a USB camera in Python 3 on Ubuntu 20.04 on ROS Noetic. **Use your own PC for this part, not the turtlebot**

**Installing the required package**:

So first let’s install the required packages:

Open a terminal on your Ubuntu 20.04. Type this command:

```bash
sudo apt-get install ros-noetic-usb-cam ros-noetic-aruco-ros
```

**Launch Files**:

In GitHub: 
https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch, there are three launch files,

1. ``usb_cam_stream_publisher.launch``: USB image publisher
   
2. ``aruco_marker_finder.launch``: single ArUco Marker finder
   
3. ``multiple_aruco_marker_finder.launch``: multiple ArUco Markers finder

**Run ROS Master**:

Open the first terminal, and run this command to execute the ROS master:

```bash
roscore
```

Let this terminal open to run the master always. Please do not close and terminate this terminal. 

**Build or compile your workspace**:

Open the second terminal, and run this command to compile your workspace:

```bash
cd catkin_ws
```

```bash
catkin_ws 
catkin_make
```

**Publishing Images from Camera and Estimating the Pose**:

Open the third terminal, and go to this folder:

```bash
cd catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/
```

and run this command to publish USB camera images and estimate the pose of the ArUco Marker:

```bash
catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch $ roslaunch usb_cam_stream_publisher.launch
```

**Finding the ArUco Marker**:

Open the fourth terminal, and go to this folder:

```bash
cd catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/
```

and run this command to find the ArUco Marker:

```bash
catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch $ roslaunch aruco_marker_finder.launch markerId:=701 markerSize:=0.05
```

**Open Rqt GUI**

Open the fifth terminal, and run this command to see the results:

```bash
rosrun rqt_gui rqt_gui
```

**Pose of ArUco Marker**

Open the sixth terminal, and run this command to see the pose of the marker:

```bash
rostopic echo /aruco_single/pose
```

Now, take the camera and keep on the ArUco Marker Id = 701 image, you can see the results in rqt_gui window (from the fifth terminal) and the sixth terminal.

Try to run multiple ArUco Markers finder code from the GitHub: 
[multi_aruco_marker](turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/multiple_aruco_marker_finder.launch)

**Reference**:

GitHub: [Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch)

## Task 3 (Optional): Find the ArUco Marker and navigate towards it. 
After managing to recognize 2 or more aruco marker at the same time, you can try using this feature to navigate your turtlebot. Check the code in [goal_pose](turtlebot3_burger_auto_navigation/auto_navigation/scripts/goal_pose.py) and try to use it for this task.

Hint: one aruco marker as the coordinate of your turtlebot, and another one as the target.

## Task 4 (Optional): Naïve obstacle avoidance using LiDAR

Follow the tutorial in [4. SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node) and [5. Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation)
 from the turtlebot offcial document

**Desired Output**: Build your own map using ```map_saver``` and use this map to achieve navigation with obstacle avoidance