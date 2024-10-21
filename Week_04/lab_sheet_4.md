# COMP0182 (Multi-Agent Systems): Lab Sheet 4

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1: Set up the catkin workspace and VS code

**1. Open the first terminal, instead of sourcing the below ROS path every time:**

source /opt/ros/noetic/setup.bash

It's better to make automate by sourcing to ~/.bashrc one time:

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

**2. Install the essential dependencies one time:**

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

After all the above steps, try to run the below command to see if the ROS works correctly or not:

roscore

Note: Please do not close this terminal as long as you working on the ROS platform.

**3. Open the second terminal, create and initialize the catkin workspace:**

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws

catkin init

**4. Clone GitHub repository:**

Go to the src folder:

cd ~/catkin_ws/src

Clone the repository:

git clone [https://github.com/Intelligent-Quads/iq_sim.git](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems.git)

**5. Build the workspace**

Go to catkin_ws, run catkin_make:

cd ~/catkin_ws

catkin_make

**6. Open the third terminal, install vs code:**

sudo snap install code --classic

**7. To open the VS code editor from the command line just run**

code .

**8. After opening the VS code editor, try to open your cloned repository folder by browsing it**
Now try to work in VS code for your programming life is easy.

## Task 2: Calibrating a Monocular Camera with ROS

ROS uses OpenCV for camera calibration but the format in which it stores the data is different than OpenCV. Also, you need to know where to place the camera calibration files so ROS can find them and publish them.

1. First, you need to install the USB cam package from ROS and uvcdynctrl to disable autofocus:

sudo apt-get install ros-noetic-usb-cam uvcdynctrl

2. Open a terminal and run roscore:

roscore

3. Turn off the autofocus of your camera (if your camera supports autofocus):

check if your camera supports autofocus:

uvcdynctrl --device=/dev/video0 --clist
turn off the autofocus:

uvcdynctrl --device=/dev/video0 --set='Focus, Auto' 0
check if the autofocus is off:

uvcdynctrl --device=/dev/video0 --get='Focus, Auto'

4. Publish the data from your camera, for example, via using usb_cam:

rosrun usb_cam usb_cam_node

5. Connect camera_calibratio to the node publishing camera images: (node and topic name should be adjusted: image:=/usb_cam/image_raw camera:=/usb_cam) and a checkerboard with 0.02517-meter squares:

rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.02517 image:=/usb_cam/image_raw camera:=/usb_cam --no-service-check

6. After getting enough images click on the calibrate and then save. If you click on the commit button it will copy calibration data into:

/home/<username>/.ros/camera_info/head_camera.yaml
 
7. Fix the calibration URL. Put the YAML file in the

/home/<username>/.ros/camera_info/head_camera.yaml

**Reference:**

https://ros-developer.com/2017/04/23/camera-calibration-with-ros/

## Task 3

## Task 4: SLAM using Laser Distance Sensor & Mapping
### Run SLAM code
1. SSH to your Turtlebot from your **Remote PC**, with "ubuntu" as username and "turtlebot" as password. Run the Bringup on **Turtlebot terminal**
```bash
ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

2. On your **Remote PC**, run ``roscore``
```bash
roscore
```

3. Still on **Remote PC**, open a new terminal and run SLAM node
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
This will take you to RViz, where the map, LiDAR, robot, etc are visulized.

### Run Teleoperation
On your **Remote PC**, run the teleoperation node, control your Turtlebot, exploring the lab and see the process of mapping
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### (Optional) Save Map
In order to let the Turtlebot auto-navigation in the scene, you need to have a global map. The map data has been collected while it is traveling in last step
```bash
rosrun map_server map_saver -f ~/map
```

### (Optional) Navigation
After you manage to get the map, try let the Turtlebot auto-navigate, following the instructions in
[5. Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes)

## Task 5: Use multiple aruco marker to navigate
In previous task, you should manage to recognize multiple aruco markers at the same time from your camera. Now combining this aruco finder function and the Turtlebot moving function, you can navigate the Turtlebot to a desired goal. The position of Turtlebot and the goal is expressed by those aruco markers separately. The combined function can be found in
[goal_pose.py](/Week_03/turtlebot3_burger_auto_navigation/auto_navigation/scripts/goal_pose.py).
Think about what else nodes you should launch before using it. 

Hint: The script subscribes two topics: ``/id100/aruco_single/pose`` and ``/id101/aruco_single/pose``, and publish ``/cmd_vel``






## To-Do List

- [x] Set up workspace/VS Code/Git Clone (Narsimlu)
- [x] Perform camera calibration (Narsimlu)
- [ ] Troubleshoot ArUco Marker/Multiple Markers using Logitech C920 Pro HD Camera (Narsimlu)
- [x] Execute SLAM using Laser Distance Sensor (Task 04 of Lab Sheet 03) Mapping (Vincent)
- [x] Set Navigation Goal Pose (Task 03 of Lab Sheet 03) (Vincent)

