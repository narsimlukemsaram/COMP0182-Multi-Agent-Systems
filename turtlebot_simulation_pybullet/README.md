# turtlebot_simulation_pybullet

## Installation

Install pybullet:

```shell
pip3 install pybullet --upgrade --user
python3 -m pybullet_envs.examples.enjoy_TF_AntBulletEnv_v0_2017may
python3 -m pybullet_envs.examples.enjoy_TF_HumanoidFlagrunHarderBulletEnv_v1_2017jul
python3 -m pybullet_envs.deep_mimic.testrl --arg_file run_humanoid3d_backflip_args.txt
```

Install necessary dependencies to run cbs:

```shell
python3 -m pip install -r requirement.txt
```

## To run the simulation

``` shell 
$ python3 muti_robot_navigation_2.py
```

