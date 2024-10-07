# COMP0182: Lab sheet 2

## Lab sheet 2: Unit test on turtlebot

### Task 1: (Optional) Install RoboStack

For some of you studying COMP0245, you should already have RoboStack installed on your computer. This is an alternative platform to Ubuntu virtual machine or dual boot. We strongly recommend you to use Ubuntu since it is more stable for ROS and Pybullet. But you can still try RoboStack since it is convenient, light and easy to install. For some parts of the script or software using in this module, there might be problems with RoboStack. If you meet some problems on happen on RoboStack and manage to fix it, please share it on moodle so that more people can use it.

1. Install conda on MacOS
    
    To get started with conda (or mamba) as package managers, you need to have a base conda installation. Please do *not* use the Anaconda installer, but rather start with [`miniforge`](https://github.com/conda-forge/miniforge) that is much more "minimal" installer. This installer will create a "base" environment that contains the package managers conda and mamba. After this installation is done, you can move on to the next steps.
    
    Download the installer using curl or wget or your favourite program and run the script.
    
    ```bash
    curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
    bash Miniforge3-$(uname)-$(uname -m).sh
    ```
    
    run `conda` or `mamba` to see if it is installed successfully.
    
2. Setting up the environment
    - Clone the `Multi-agent-system` repository
        
        ```bash
        git clone https://github.com/VModugno/Multi_agent_system.git
        ```
        
3. Install the Conda Environment:
    
    The environment_multi_agent_sys.yaml file includes all the dependencies required. To create the environment, run the below commands.
    

```bash
mamba env create -f environment_multi_agent_sys.yaml
mamba activate turtle_bot_env # or if it does not work try conda activate turtle_bot_env
```

Then the environment will change from `base` to `turtle_bot_env`.

![image.png](COMP0182%20Lab%20sheet%202%20117a981a2de680998d1bc94c157ed27a/image.png)

### Task 2: Unit test in pybullet

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