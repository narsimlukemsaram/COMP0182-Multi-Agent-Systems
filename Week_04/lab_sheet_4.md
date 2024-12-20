# COMP0182 (Multi-Agent Systems): Lab Sheet 4

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1: Perform Camera calibration (using Checkerboard)

ROS uses OpenCV for camera calibration but the format in which it stores the data differs from OpenCV. Also, you need to know where to place the camera calibration files so ROS can find and publish them (/home/<username>/.ros/camera_info/head_camera.yaml).

First, you can connect the Logitech C920 HD Pro camera to your laptop.

1. Open the first terminal, and run the ROS master:

```bash
roscore
```

Please keep this terminal open.

2. Open the second terminal, install the USB cam package from ROS and uvcdynctrl to disable autofocus, open a terminal, and run the below:

```bash
sudo apt-get install ros-noetic-usb-cam uvcdynctrl
```

3. Identify the camera:

First, plug the camera into the USB port.

Open a terminal and list all cameras plugged in and detected by the system:

```
ls /dev/video*
```

Plug and unplug the Logitech C920 HD Pro camera and identify the correct /dev/video*.

In my case, the camera was mounted on path /dev/video2.

4. Now, check if your camera autofocus status:

```bash
uvcdynctrl --device=/dev/video2 --clist

Listing available controls for device /dev/video2:
  Brightness
  Contrast
  Saturation
  White Balance Temperature, Auto
  Gain
  Power Line Frequency
  White Balance Temperature
  Sharpness
  Backlight Compensation
  Exposure, Auto
  Exposure (Absolute)
  Exposure, Auto Priority
  Pan (Absolute)
  Tilt (Absolute)
  Focus (absolute)
  Focus, Auto
  Zoom, Absolute
```

5. If it is enabled, you can turn off the autofocus:

```bash
uvcdynctrl --device=/dev/video2 --set='Focus, Auto' 0
```

check if the autofocus is off or not:

```bash
uvcdynctrl --device=/dev/video2 --get='Focus, Auto'
```

It should return 0.

6. Now, install camera calibration:

```bash
rosdep install camera_calibration
```

7. Publish the data from your camera, for example, via using usb_cam:

```bash
rosrun usb_cam usb_cam_node
```

8. In the third terminal, run rostopic list:

```bash
rostopic list

/rosout
/rosout_agg
/usb_cam/camera_info
/usb_cam/image_raw
/usb_cam/image_raw/compressed
/usb_cam/image_raw/compressed/parameter_descriptions
/usb_cam/image_raw/compressed/parameter_updates
/usb_cam/image_raw/compressedDepth
/usb_cam/image_raw/compressedDepth/parameter_descriptions
/usb_cam/image_raw/compressedDepth/parameter_updates
/usb_cam/image_raw/theora
/usb_cam/image_raw/theora/parameter_descriptions
/usb_cam/image_raw/theora/parameter_updates
```

9. Open the fourth terminal, and connect camera_calibration to the node publishing camera images:
   (node and topic name should be adjusted: image:=/usb_cam/image_raw
                                            camera:=/usb_cam) and a checkerboard with
                                            0.02517-meter squares:

```bash
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.02517 image:=/usb_cam/image_raw camera:=/usb_cam --no-service-check
```

10. After getting enough images, click on the calibrate and then save. If you click on the commit button it will copy calibration data into:

/home/[username]/.ros/camera_info/head_camera.yaml
 
11. Fix the calibration URL. Put the YAML file in the

/home/[username]/.ros/camera_info/head_camera.yaml

Note: Try to keep the checkerboard slightly oriented (45/90/135/180 degrees) and change it from clockwise to anti-clockwise during the calibration process. 

**References:**

[1]. https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration/

[2]. https://ros-developer.com/2017/04/23/camera-calibration-with-ros/


## Task 2: Goal Pose using ArUco Markers (using USB Camera/Webcam)

In the previous lab session, you should have managed to recognize single/multiple ArUco markers from your camera. Now combining this aruco finder function and the TurtleBot3 moving function, you can navigate the TurtleBot3 to a desired goal. The position of TurtleBot3 and the goal is expressed by those ArUco markers separately. The combined function can be found in
[goal_pose.py](turtlebot3_burger_auto_navigation/auto_navigation/scripts/goal_pose.py).

Think about what other nodes you should launch before using it. 

Hint: The script subscribes two topics: ``/id100/aruco_single/pose`` and ``/id101/aruco_single/pose``, and publish ``/cmd_vel``, and when you want to ``rosrun`` a node, you will need to give executable permission to it.

Desired outcome: successfully navigate your Turtlebot to a goal position.

## Task 3: SLAM/Map using Laser Distance Sensor and execute Goal Pose (using LDS/LiDAR)

### Run SLAM code

SSH to your TurtleBot3 from your **Remote PC**, with "**ubuntu**" as username and "**turtlebot**" as password. Run the Bringup on **TurtleBot3 terminal**
```bash
ssh ubuntu@{IP_ADDRESS_OF_TURTLEBOT3}
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

On your **Remote PC**, run ``roscore``
```bash
roscore
```

Still on **Remote PC**, open a new terminal and run the SLAM node
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
This will take you to RViz, where the map, LiDAR, robot, etc are visualized.

### Run Teleoperation
On your **Remote PC**, run the teleoperation node, control your TurtleBot3, explore the lab, and see the process of mapping

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Press a, w, x, d to move TurtleBot3 around the lab. At the end, press s to stop the TurtleBot3.

### Save Map

In order to let the TurtleBot3 auto-navigation in the scene, you need to have a global map. The map data was collected while it is traveling in the last step:

```bash
rosrun map_server map_saver -f ~/map
```

The -f option specifies a folder location and a file name where files are to be saved. With the above command, map.pgm and map.yaml will be saved in the home folder ~/(/home/${username}). 

After you manage to get the map, try let the TurtleBot3 auto-navigate, following the instructions in
[5. Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes). 

## Task 4: Obstacle Avoidance during navigation (using LDS/LiDAR/Camera)

The naive algorithm can be found in [Naive_Obstacle_Avoidance](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/turtlebot3_burger_auto_navigation/naive_obstacle_avoidance/). 

Note:

Don't forget to change the tab to ``noetic``

==============================================================================

## Troubleshoot / Suggestions

==============================================================================

**Install usb_cam ROS Package**

1. The usb_cam package is commonly used in ROS to interface with USB cameras or webcams.

```bash
sudo apt install ros-noetic-usb-cam
```

**To view the camera feed in ROS, you can use rqt_image_view:**

2. Install the necessary tools:

```bash
sudo apt install ros-noetic-rqt-image-view
```

3. Run the image viewer:

```bash
rqt_image_view
```

In the viewer, select the topic /usb_cam/image_raw to see the camera feed.

## If you encounter any dependency issues, refer to the steps mentioned below to resolve unmet dependencies.

If the USB or webcam feed did not display in the above steps, there are several troubleshooting steps you can follow to identify and resolve the issue.

1. Check Camera Detection
   
First, ensure that the USB or webcam is properly detected by your system.

a. Use v4l2-ctl to list video devices:

```bash
v4l2-ctl --list-devices
```

This command should list all connected video devices, including the webcam. 

If /dev/video2 is not listed, try checking other video devices like /dev/video0, /dev/video1, etc.

b. Check for video device files:

```bash
ls /dev/video*
```

This should display video devices like /dev/video0, /dev/video1, etc. If no devices are listed, your system may not be detecting the webcam properly.

2. Check Webcam Permissions
   
If your webcam is detected but not accessible, there could be a permission issue.

a. Add your user to the video group:

Ensure your user has the necessary permissions to access video devices.

```bash
sudo usermod -aG video $USER
```

After running this command, log out and log back in for the changes to take effect.

3. Check for Camera Driver Issues

Sometimes, webcam drivers may not be properly loaded.

a. Install the v4l-utils package:

```bash
sudo apt install v4l-utils
```

b. Verify the camera status using dmesg:

Run the following command right after plugging in your USB camera (or checking the built-in webcam):

```bash
dmesg | grep video
```

This will show kernel logs related to video devices, which might indicate if there are any issues with loading the camera driver.

4. Install or Reinstall Camera Software

Ensure that the software required for your webcam is installed and working.

a. Install Cheese:

If Cheese failed to open the webcam feed, try reinstalling it:

```bash
sudo apt install cheese
```

Then, run cheese again:
```bash
cheese
```

5. Test with ffmpeg or mplayer

To bypass GUI tools and directly test the camera feed from the terminal, you can use ffmpeg or player.

a. Test with ffmpeg:

```bash
ffmpeg -f v4l2 -i /dev/video2 -vframes 1 out.jpg
```

This command will capture a single frame from the webcam and save it as out.jpg in the current directory. If successful, it means the camera is working.

b. Test with player:

Install mplayer and test it:

```bash
sudo apt install player
```

```bash
mplayer tv:// -tv device=/dev/video2
```

This should open a window with your webcam feed. Adjust device=/dev/videoX if your camera is on a different device.

6. Check for Hardware Issues

If none of the above steps work, there might be a hardware-related issue with your webcam or USB port:

 Try using a different USB port (if using a USB camera).
 
 Test the webcam on another machine or operating system to rule out hardware failure.

7. Update System Drivers

Ensure that all drivers are up to date:

```bash
sudo apt update
```

```bash
sudo apt upgrade
```

After running the updates, reboot your system and check if the webcam feed works.

8. To disable or modify the use of focus_auto in your ROS usb_cam launch file or camera configuration, follow these steps:

In the launch file (e.g., usb_cam_stream_publisher.launch), you need to adjust the parameters that control the camera settings.

First, open the launch file:

```bash
nano usb_cam_stream_publisher.launch
```

Add a parameter to explicitly disable focus_auto. Use v4l2 control parameters, which can be set in the launch file:

```bash
<param name="focus_auto" value="0" /> <!-- 0 means 'off' -->
```

Example of how your node might look after adding the focus control:

```bash
<node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
  <param name="video_device" value="/dev/video2" />
  <param name="image_width" value="640" />
  <param name="image_height" value="480" />
  <param name="pixel_format" value="mjpeg" />
  <param name="camera_frame_id" value="usb_cam" />
  <param name="io_method" value="mmap" />
  <param name="focus_auto" value="0" /> <!-- Disable autofocus -->
</node>
```

After modifying the launch file or using v4l2-ctl, restart the launch file with:

```bash
roslaunch usb_cam_stream_publisher.launch
```

This should prevent the error related to focus_auto if it's unsupported by your camera. If the focus_auto parameter isn't recognized, you can leave it out, as the node will ignore it.

9. Adjust the Pixel Format:

If the camera doesn't support mjpeg as the pixel format, try changing it to yuyv, which is widely supported:

```bash
<param name="pixel_format" value="yuyv"/>
```

Save the launch file and try again:

```bash
roslaunch usb_cam_stream_publisher.launch
```

## How to set up the catkin workspace and VS code

1. Regularly need to ensure your system is up-to-date and has the necessary packages for ROS:

```bash
sudo apt update
```

```bash
sudo apt upgrade
```

2. Open the first terminal, instead of sourcing the below ROS path every time:

```bash
source /opt/ros/noetic/setup.bash
```

It's better to make automate by sourcing to ~/.bashrc one time:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

```bash
source ~/.bashrc
```

3. Install the essential dependencies one time:

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

4. After all the above steps, try to run the below command to see if the ROS works correctly or not:

```bash
roscore
```

Note: Please do not close this terminal as long as you working on the ROS platform.

5. Open the second terminal, create and initialize the catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
```

```bash
cd ~/catkin_ws
```

```bash
catkin init
```

6. Clone GitHub repository:

Go to the src folder:

```bash
cd ~/catkin_ws/src
```

Clone the repository:

```bash
git clone https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems.git
```

7. Build the workspace

Go to catkin_ws, run catkin_make:

```bash
cd ~/catkin_ws
```

```bash
catkin_make
```

8. Open the third terminal, install vs code:

```bash
sudo snap install code --classic
```

9. To open the VS code editor from the command line just run:

```bash
code .
```

10. After opening the VS code editor, try to open your cloned repository folder by browsing it

Now try to work in VS code for your programming life is easy.

## To-Do List

- [Task 1] Perform camera calibration (using Checkerboard)
- [Task 2] Goal Pose using ArUco Markers (using USB Camera/Webcam)
- [Task 3] SLAM/Map using Laser Distance Sensor and execute Goal Pose (using LDS/LiDAR)
- [Task_4] Obstacle Avoidance (using LDS/LiDAR/Camera)

## Troubleshooting References

- Set up workspace/VS Code/Git Clone
- Troubleshoot ArUco Marker/Multiple Markers using Logitech C920 Pro HD Camera
