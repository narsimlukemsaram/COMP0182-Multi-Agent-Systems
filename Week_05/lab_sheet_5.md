# COMP0182 (Multi-Agent Systems): Lab Sheet 5

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1: Perform real single-robot planning/navigation

First, you can connect the Logitech C920 HD Pro camera to your laptop.

1. Open the first terminal, and run the ROS master:

```bash
roscore
```

Please keep this terminal open.

 

**References:**

[1]. https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration/



## Task 2: Naive obstacle avoidance using LDS-02

To run naive obstacle avoidance on a single turtlebot using LDS-02, please follow the below instructions: 

1. Open the first terminal on **Remote PC**, and run the ROS master:

```bash
roscore
```

Please keep this terminal open.

2. Open second terminal, SSH to your TurtleBot3 from **Remote PC**, with "**ubuntu**" as username and "**turtlebot**" as password.
  
Run the Bringup on **TurtleBot3 terminal**:

```bash
ssh ubuntu@{IP_ADDRESS_OF_TURTLEBOT3}
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

3. Still on **Remote PC**, open third terminal and run the naive obstacle avoidance node:
   
```bash
export TURTLEBOT3_MODEL=burger
rosrun naive_obstacle_avoidance naive_obstacle_avoidance_node
```

### Run Teleoperation

On your **Remote PC**, run the teleoperation node, control your TurtleBot3, try to keep some obstacles in front of TurtleBot3, and see the obstacle avoidance:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Press a, w, x, d to move TurtleBot3 around the lab. At the end, press s to stop the TurtleBot3.


**References:**

[1] https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/naive_obstacle_avoidance/ 



## Task 3: Auto navigation

In auto navigation, the camera detect the marker on top of the robot and the marker on the ground. The marker on the ground is the goal position and the application allow the robot to navigate from its original position to the goal position.

Firstly, the id number of ArUco markers being used need to be remember. In the example, marker on robot has ID 100 and marker on the ground has ID 101. Different ID's can be used with modification in the code in order to correctly detect and read marker positions.

1. Open the first terminal on **Remote PC**, and run the ROS master:

```bash
roscore
```

Please keep this terminal open.

2. Open second terminal, SSH to your TurtleBot3 from **Remote PC**, with "**ubuntu**" as username and "**turtlebot**" as password.
  
Run the Bringup on **TurtleBot3 terminal**:

```bash
ssh ubuntu@{IP_ADDRESS_OF_TURTLEBOT3}
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

3. Still on **Remote PC**, open third terminal and run multiple aruco marker finder:
   
```bash
export TURTLEBOT3_MODEL=burger
roslaunch auto_aruco_marker_finder multiple_aruco_marker_finder.launch
```

4. Still on **Remote PC**, open fourth terminal and run rqt to check if markers are correctly detected:

```bash
export TURTLEBOT3_MODEL=burger
rosrun rqt_gui rqt_gui
```

The rqt window shows a correct detection.

5. Still on **Remote PC**, open fifth terminal and run:

```bash
export TURTLEBOT3_MODEL=burger
rosrun auto_navigation goal_pose.py
```

Then the robot should be moving to the destination.

If the robot is not moving and error occurs on the terminal, shutdown all terminal and repeat from step 1.

**References:**

[1] https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_navigation/.


## [Homework]: Multi-robot obstacle avoidance

## To-Do List

- [Task 1] Perform real single-robot planning/navigation
- [Task 2] Naive obstacle avoidance using LDS-02
- [Task 3] Auto navigation
- [Homework] Multi-robot obstacle avoidance 
