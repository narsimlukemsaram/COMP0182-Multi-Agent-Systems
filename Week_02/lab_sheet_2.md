# COMP0182 (Multi-Agent Systems): Lab Sheet 2

## Lab Sheet 2: Unit Test on TurtleBot3 in PyBullet Simulation

### Task 1: Unit Test in PyBullet Simulation

Same as shown in the TurtleBot3 Tutorial slide, try to do the unit test as below:

1. Clone the repository
    Go to your home directory and clone the PyBullet simulation code using the web URL:
    ```bash
    git clone https://github.com/JacksonChiy/turtlebot_simulation_pybullet
    ```
    
2. Setup the environment
    
    ```bash

    # Go to the PyBullet directory:
    cd turtlebot_simulation_pybullet

    # Install PyBullet:
    pip3 install pybullet
    
    #Install necessary dependencies: 
    python3 -m pip install -r requirement.txt
    
    ```
    
3. Run a single TurtleBot3 motion control test:
    
    ```bash

    ~/turtlebot_simulation_pybullet# python3 single_bot_motion_control.py
    
    ```
    
    Try to run the remaining tests,
        "multi_robot_navigation.py"
        "multi_robot_navigation_2.py,
        "multi_robot_navigation_3.py",
        "multi_robot_navigation_deliver.py", and
        "final_challenge.py" as well!
    

### Known Problem

1. ~~"multi_robot_navigation_2.py" on all platforms (Windows/macOS): crashed.~~
2. "multi_robot_navigation_2.py", "multi_robot_navigation_3.py", "multi_robot_navigation_deliver.py", "final_challenge.py" on macOS while using RoboStack: Python crashed.

**Reference**: 
[https://github.com/JacksonChiy/turtlebot_simulation_pybullet](https://github.com/JacksonChiy/turtlebot_simulation_pybullet)
