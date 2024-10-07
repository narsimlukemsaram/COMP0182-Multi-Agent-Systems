# COMP0182: Lab sheet 2

## Lab sheet 2: Unit test on turtlebot

### Task 1: Unit test in pybullet

Same as shown in the Turtlebot Tutorial slide, try to do the unit test now.

1. Clone the repository
    
    ```bash
    git clone https://github.com/JacksonChiy/turtlebot_simulation_pybullet
    ```
    
2. Setup the environment
    
    ```bash
    cd “Directory name”
    python3 -m pip install -r requirement.txt
    ```
    
3. Run motion control test
    
    ```bash
    python3 single_bot_motion_control.py
    ```
    
    Try `multi_robot_navigation_*` and `final_challenge` as well!
    

### Known Problem

1. `multi_robot_navigation_2` on all platform: crashed
2. `multi_robot_navigation_2` ,`multi_robot_navigation_3`, `multi_robot_navigation_deliver` , `final_challenge` on MacOS while using RoboStack: Python crashed

Reference: 

[https://github.com/conda-forge/miniforge](https://github.com/conda-forge/miniforge)