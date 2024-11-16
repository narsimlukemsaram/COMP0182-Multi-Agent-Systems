# COMP0182 (Multi-Agent Systems): Lab Sheet 8

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1: Single-robot auto navigation toward one destination marked by one ArUco Marker (without obstacles)

Again, the target is to finish running single-robot (real TurtleBot3) auto navigation toward one destination marked by one ArUco marker from the previous lab session. 

All students should have a camera calibrated and be able to detect a single ArUco marker. 

If not, please do it and ensure you can run the auto navigation script with one ArUco marker. One marker is on the TurtleBot3, and the other is the destination.

Once you finish this, you can move your robot autonomously to the destination ArUco marker. Here are steps:

1. First, Plug and unplug the Logitech C920 HD Pro camera into the USB port and identify the correct /dev/video*.

Open a terminal and list all cameras plugged in and detected by the system:

```
ls /dev/video*
```

In my case, the camera was mounted on path /dev/video**2**.

2. Open the first terminal and run the roscore.

```
roscore
```

Keep this terminal open and let it run the roscore (master).

Ensure the ArUco marker 100 is attached on top of your TyrtleBot3 (with correct orientation) and ArUco marker 101 is a bit far away from your TurtleBot3 on the floor. 

3. Open the second terminal, launch multiple ArUco marker finder:

```
   roslaunch auto_aruco_marker_finder multiple_aruco_marker_finder.launch
```

Update the correct USB port for your camera in <catkin_ws>/COMP0182-Multi-Agent-Systems/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/multiple_aruco_marker_finder.launch. In my case, the camera was mounted on path /dev/video**2**. 

4. Open the third terminal, and run the rqt gui for visualizing the detections:

```
   rosrun rqt_gui rqt_gui
```

In rqt GUI, select /id100/aruco_single/result, /id101/aruco_single/result, etc.

If the ArUco markers (100 and 101) are not visible, keep them in the field of view of camera).

Output like this:

![image.png](imgs/SingleArucoMarker.png)

5. Open the fourth terminal, and connect to your TurtleBot3, using ssh:
   
On your turtlebot3, run bringup:

```
**ubuntu@ubuntu2004:~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

5. Open the fifth terminal, and run the auto navigation script (goal_pose.py):

```
   rosrun auto_navigation goal_pose.py
```

The TurtleBot3 (ArUco marker 100) goes to the destination (ArUco marker 101).

## Task 2: Single-robot auto navigation toward one destination marked by one ArUco Marker (with obstacles)

To be filled.

## Task 3: Single-robot auto navigation fetch and delivery in PyBullet Simulation

To be filled.

## Task 4 (Optional): Install and run YOLOv8 object detection on TurtleBot3 (connect camera to Raspberry Pi 4B) using the Logitech C920 HD Pro camera

To be filled.

==============================================================================

## Troubleshoot / Suggestions

==============================================================================

**Step 1. Setup and make Ubuntu 20.04 identify the camera**.

First plug the camera to USB port.

Open a terminal, list all cameras plugged in and detected by system:

```bash
ls /dev/video*
```

Plug and unplug the Logitech C920 HD Pro camera and identify the correct /dev/video*.

In my case camera was mounted on path **/dev/video2**.

**Step 2. Open the usb_cam_stream_publisher.launch file and change your identified camera.**

Go to the folder and open the usb_cam_stream_publisher.launch file: <catkin_ws>/src/COMP0182-Multi-Agent-Systems/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/usb_cam_stream_publisher.launch. 

Example of how your node might look after changing video and changing the auto focus control:

```bash
<!--
Example of run:
roslaunch usb_cam_stream_publisher.launch video_device:=/dev/video2 image_width:=640 image_height:=480
-->

<launch>
<arg name="video_device" default="/dev/video2" /> <!-- video2 for Logitech C920 HD Pro Camera  -->
<arg name="image_width" default="640" />
<arg name="image_height" default="480" />


<node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
	<param name="video_device" value="$(arg video_device)" />
	<param name="image_width" value="$(arg image_width)" />
	<param name="image_height" value="$(arg image_height)"/>
	<param name="pixel_format" value="mjpeg" />
	<param name="camera_frame_id" value="usb_cam" />
	<param name="io_method" value="mmap"/>
  <param name="focus_auto" value="0" />  <!-- Disable autofocus -->
</node>
</launch>
```

**References:**

[1] Setup webcamera with ROS on Ubuntu 20.04, https://medium.com/@sigmoid90/ros-tips-setup-camera-with-ros-on-ubuntu-20-04-0069aea0341f/.

## To-Do List

- [Task 1] Single-robot auto navigation toward one destination marked by one ArUco Marker (without obstacles)
- [Task 2] Single-robot auto navigation toward one destination marked by one ArUco Marker (with obstacles)
- [Task 3] Single-robot auto navigation fetch and delivery in PyBullet Simulation
- [Task 4][Optional] Install and run YOLOv8 object detection on TurtleBot3 (connect camera to Raspberry Pi 4B) using the Logitech C920 HD Pro camera
