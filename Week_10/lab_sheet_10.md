# COMP0182 (Multi-Agent Systems): Lab Sheet 10

## Final Student Challenge: Fetch and Delivery with Two TurtleBots

In this final challenge, the focus is on implementing a **fetch and delivery task** using two TurtleBots in a real-world arena with obstacles (walls). The goal is to plan collision-free trajectories for the TurtleBots to complete the task efficiently. 

### Objective
- **Minimize delivery time** while ensuring the following:
  - Both TurtleBots visit the fetch points (FPs) and reach their respective goal positions.
  - Avoid collisions with walls or between the TurtleBots.

An example solution has been provided in the last week's assignment. However, it has **poor performance** and can be significantly improved.

---

## Task Overview

### Setup
- **Environment**: Fixed maze structure with predefined starting (`start1`, `start2`) and goal positions (`goal1`, `goal2`) for the TurtleBots.
- **TurtleBots**: Two robots navigating simultaneously in the arena with obstacles.

### Task Steps
1. **Initial Requirements**:
   - Both TurtleBots must begin at their respective starting positions.
   - They must navigate to touch the fetch points (FPs) and then move to their goal positions.

2. **Penalties**:
   - **Collision with walls**: Adds a **10-second penalty** for each instance.
   - **Collision between TurtleBots**: Adds a **20-second penalty** for each instance.

3. **Attempts**:
   - Each student has **two attempts** to complete the task.
   - If both attempts are successful, the **best performance** will be used as the final result (not the average).

4. **Mission Start and End**:
   - The mission starts when the TurtleBots begin their trajectory.
   - It ends when both TurtleBots stop at their respective goal positions.

---

### Implementation
1. **Simulated Environment**:
   - Students must first implement their solution in **PyBullet simulation** and demonstrate it before attempting the real-world challenge.
   - Use **CBS, PBS, RRT***, or a **student-designed algorithm** for multi-agent navigation.
   - Save the resulting navigation trajectory to a file.

2. **Real-World Challenge**:
   - Replay the saved trajectory file in the real-world arena.
   - The TurtleBots will use waypoint-based navigation derived from the simulation.

---

## Key Points to Remember
- **Collision-free navigation** is critical.
- Efficient trajectories will minimize fetch and delivery time.
- The best result out of the two attempts will be considered as the final performance.

---

Good luck with the challenge! 🚀
