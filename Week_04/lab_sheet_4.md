# COMP0182 (Multi-Agent Systems): Lab Sheet 4

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1 
## Task 2
## Task 3

## Task 4 SLAM using Laser Distance Sensor & Mapping
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

- [ ] Set up workspace/VS Code/Git Clone (Narsimlu)
- [ ] Perform camera calibration (Narsimlu)
- [ ] Troubleshoot ArUco Marker/Multiple Markers using Logitech C920 Pro HD Camera (Narsimlu)
- [x] Execute SLAM using Laser Distance Sensor (Task 04 of Lab Sheet 03) Mapping (Vincent)
- [x] Set Navigation Goal Pose (Task 03 of Lab Sheet 03) (Vincent)

