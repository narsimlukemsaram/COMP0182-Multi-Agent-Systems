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

The naive algorithm can be found in [Naive_Obstacle_Avoidance](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/naive_obstacle_avoidance/). 

## Task 3: Auto navigation

In the previous lab session, you should have managed to recognize single/multiple ArUco markers from your camera. Now combining this aruco finder function and the TurtleBot3 moving function, you can navigate the TurtleBot3 to a desired goal. The position of TurtleBot3 and the goal is expressed by those ArUco markers separately. The combined function can be found in
[goal_pose.py](/Week_03/turtlebot3_burger_auto_navigation/auto_navigation/scripts/goal_pose.py).

Think about what other nodes you should launch before using it. 

Hint: The script subscribes two topics: ``/id100/aruco_single/pose`` and ``/id101/aruco_single/pose``, and publish ``/cmd_vel``, and when you want to ``rosrun`` a node, you will need to give executable permission to it.

Desired outcome: successfully navigate your Turtlebot to a goal position.

## [Homework]: Multi-robot obstacle avoidance

## To-Do List

- [Task 1] Perform real single-robot planning/navigation
- [Task 2] Naive obstacle avoidance using LDS-02
- [Task 3] Auto navigation
- [Homework] Multi-robot obstacle avoidance 
