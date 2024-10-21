# COMP0182 (Multi-Agent Systems): Lab Sheet 4

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1: Set up the catkin workspace and VS code

Regularly need to ensure your system is up-to-date and has the necessary packages for ROS:

sudo apt update

sudo apt upgrade

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

## Task 3: Troubleshoot ArUco Marker/Multiple Markers using USB Camera

**Install usb_cam ROS Package**

The usb_cam package is commonly used in ROS to interface with USB cameras or webcams.

sudo apt install ros-noetic-usb-cam

**To view the camera feed in ROS, you can use rqt_image_view:**

Install the necessary tools:

sudo apt install ros-noetic-rqt-image-view

Run the image viewer:

rqt_image_view

In the viewer, select the topic /usb_cam/image_raw to see the camera feed.

----------------------------------------------------------------------------------------------------------
If you encounter any dependency issues, refer to the steps mentioned below to resolve unmet dependencies.

If the USB or webcam feed did not display in the above steps, there are several troubleshooting steps you can follow to identify and resolve the issue.

**1. Check Camera Detection**
First, ensure that the USB or webcam is properly detected by your system.

a. Use v4l2-ctl to list video devices:

v4l2-ctl --list-devices

This command should list all connected video devices, including the webcam. 

If /dev/video0 is not listed, try checking other video devices like /dev/video1, /dev/video2, etc.

b. Check for video device files:

ls /dev/video*

This should display video devices like /dev/video0, /dev/video1, etc. If no devices are listed, your system may not be detecting the webcam properly.

**2. Check Webcam Permissions**
   
If your webcam is detected but not accessible, there could be a permission issue.

a. Add your user to the video group:

Ensure your user has the necessary permissions to access video devices.

sudo usermod -aG video $USER

After running this command, log out and log back in for the changes to take effect.

**3. Check for Camera Driver Issues**

Sometimes, webcam drivers may not be properly loaded.

a. Install the v4l-utils package:

sudo apt install v4l-utils

b. Verify the camera status using dmesg:

Run the following command right after plugging in your USB camera (or checking the built-in webcam):

dmesg | grep video

This will show kernel logs related to video devices, which might indicate if there are any issues with loading the camera driver.

**4. Install or Reinstall Camera Software**

Ensure that the software required for your webcam is installed and working.

a. Reinstall Cheese:

If Cheese failed to open the webcam feed, try reinstalling it:

sudo apt remove cheese

sudo apt install cheese

Then, run cheese again:

cheese

**5. Test with ffmpeg or mplayer**

To bypass GUI tools and directly test the camera feed from the terminal, you can use ffmpeg or player.

a. Test with ffmpeg:

ffmpeg -f v4l2 -i /dev/video0 -vframes 1 out.jpg

This command will capture a single frame from the webcam and save it as out.jpg in the current directory. If successful, it means the camera is working.

b. Test with player:

Install mplayer and test it:

sudo apt install player

mplayer tv:// -tv device=/dev/video0

This should open a window with your webcam feed. Adjust device=/dev/videoX if your camera is on a different device.

**6. Check for Hardware Issues**

If none of the above steps work, there might be a hardware-related issue with your webcam or USB port:

 Try using a different USB port (if using a USB camera).
 
 Test the webcam on another machine or operating system to rule out hardware failure.

**7. Update System Drivers**

Ensure that all drivers are up to date:

sudo apt update

sudo apt upgrade

After running the updates, reboot your system and check if the webcam feed works.

**8. To disable or modify the use of focus_auto in your ROS usb_cam launch file or camera configuration, follow these steps:**

In the launch file (e.g., usb_cam_stream_publisher.launch), you need to adjust the parameters that control the camera settings.

First, open the launch file:

nano usb_cam_stream_publisher.launch

Add a parameter to explicitly disable focus_auto. Use v4l2 control parameters, which can be set in the launch file:

<param name="focus_auto" value="0" /> <!-- 0 means 'off' -->

Example of how your node might look after adding the focus control:

<node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
  <param name="video_device" value="/dev/video0" />
  <param name="image_width" value="640" />
  <param name="image_height" value="480" />
  <param name="pixel_format" value="mjpeg" />
  <param name="camera_frame_id" value="usb_cam" />
  <param name="io_method" value="mmap" />
  <param name="focus_auto" value="0" /> <!-- Disable autofocus -->
</node>

After modifying the launch file or using v4l2-ctl, restart the launch file with:

roslaunch usb_cam_stream_publisher.launch

This should prevent the error related to focus_auto if it's unsupported by your camera. If the focus_auto parameter isn't recognized, you can leave it out, as the node will ignore it.

**9. Ti disable manually the use of focus_auto, follow these steps:**

 You can use the v4l2-ctl command to check if the camera supports this control and what its current value is. Run:

 v4l2-ctl --list-ctrls

 This will give you a list of all the controls supported by your camera. If focus_auto is present, you will see something like:

 focus_auto (menu)   : min=0 max=1 default=1 value=1

 where, 
 
        0 means autofocus is disabled. 
 
       1 means autofocus is enabled.

You can manually disable it using the command:

v4l2-ctl --set-ctrl=focus_auto=0


## Task 4: SLAM using Laser Distance Sensor & Mapping
### Run SLAM code
1. SSH to your Turtlebot from your **Remote PC**, with "**ubuntu**" as username and "**turtlebot**" as password. Run the Bringup on **Turtlebot terminal**
```bash
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
export TURTLEBOT3_MODEL=burger
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
- [x] Troubleshoot ArUco Marker/Multiple Markers using Logitech C920 Pro HD Camera (Narsimlu)
- [x] Execute SLAM using Laser Distance Sensor (Task 04 of Lab Sheet 03) Mapping (Vincent)
- [x] Set Navigation Goal Pose (Task 03 of Lab Sheet 03) (Vincent)

