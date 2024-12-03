# COMP0182 (Multi-Agent Systems): Lab Sheet 10

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Final Student Challenge

Fetch and delivery in the real-world arena with two TurtleBots.

In this final challenge task, we focus on fetch and delivery with two TurtleBots with obstacles (walls) in the real-world arena. The task is to plan collision-free trajectories of the TurtleBots with fetch points. Both TurtleBots have predefined start and fetch positions. The objective of the task is to minimize the delivery time while touching the fetching points and not colliding the TurtleBots with the walls or each other. An already working solution has been provided as a part of the assignment in the last week. However, this example solution has poor performance and can be improved significantly.

# Task Overview

You are given two TurtleBots required to inspect fetch points (FPs) as fast as possible in an arena environment with obstacles. Your task is to start your TurtleBots from the start, touch the FPs, and reach goal positions while keeping a safe distance from the obstacles and between the two TurtleBots. 

You have two run attempts. If you TurtleBot touches the walls, then it is a 10-second penalty. If you touch the TurtleBots while running the mission, then it is a 20-second penalty. The overall objective is to minimize the fetch and delivery time of both TurtleBots.

The trajectories are required to begin and end at predefined starting locations. The mission starts when the trajectory following starts and ends once the TurtleBots stop at their goal positions.

It will be defined as a fixed maze structure with the corresponding start1/start2 and goal1/goal2 positions for the robots in the maze. 

Students must implement the solution in PyBullet simulation and show us before going to real-challenge. 

Next, the students must run the multi-agent auto navigation in a PyBullet simulation using two TurtleBots with CBS/PBS/RRT* (or a student innovative algorithm) and save the resulting navigation trajectory to a file.

Run the same multi-agent navigation of the two robots simultaneously using the waypoints based on the saved trajectory file in the real-world arena.
