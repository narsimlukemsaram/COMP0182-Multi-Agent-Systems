import pybullet as p
import time
import pybullet_data
import yaml
from cbs import cbs
import math

def createBoundaries(length, width):
    """
        create rectangular boundaries with length and width

        Args:

        length: integer

        width: integer
    """
    for i in range(length):
        p.loadURDF("data/rectangle_horizontal.urdf", [i, -1, 0.5])
        p.loadURDF("data/rectangle_horizontal.urdf", [i, width, 0.5])
    for i in range(width):
        p.loadURDF("data/rectangle_vertical.urdf", [-1, i, 0.5])
        p.loadURDF("data/rectangle_vertical.urdf", [length, i, 0.5])
    p.loadURDF("data/rectangle_corner.urdf", [length, -1, 0.5], p.getQuaternionFromEuler([0, 0, -math.pi/2]))
    p.loadURDF("data/rectangle_corner.urdf", [length, width, 0.5])
    p.loadURDF("data/rectangle_corner.urdf", [-1, width, 0.5], p.getQuaternionFromEuler([0, 0, math.pi/2]))
    p.loadURDF("data/rectangle_corner.urdf", [-1, -1, 0.5], p.getQuaternionFromEuler([0, 0, math.pi]))

def checkPosWithBias(Pos, goal, bias):
    """
        Check if pos is at goal with bias

        Args:

        Pos: Position to be checked, [x, y]

        goal: goal position, [x, y]

        bias: bias allowed

        Returns:

        True if pos is at goal, False otherwise
    """
    if(Pos[0] < goal[0] + bias and Pos[0] > goal[0] - bias and Pos[1] < goal[1] + bias and Pos[1] > goal[1] - bias):
        return True
    else:
        return False

def check_goal_reached(agents,goals):
    """
        For each robot, check if the ultimate goal position is reached

        Args:

        agents: array containing the boxID for each agent

        goals: dictionary with boxID as the key and the corresponding goal positions as values

        Returns:

        True if all robots have arrived at their goal position, False otherwise.
    """
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        goal = goals[i]
        if(not checkPosWithBias(pos, goal, 0.4)):
            return False
    return True

def check_next_reached(agents, schedule, index):
    """
        For each robot, check if the next position in their path is reached

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        index: index of the current position in the path.

        Returns:

        True if all robots have arrived at their next position, False otherwise.
    """
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        if(index >= len(schedule[i])):
            continue
        if (not checkPosWithBias(pos, [schedule[i][index]["x"], schedule[i][index]["y"]], 0.4)):
            return False
    return True

def set_velocity(agent, schedule, index):
    """
        Set velocity for robots to follow the path in the schedule.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        index: index of the current position in the path.

        Returns:

        Leftwheel and rightwheel velocity.
    """
    speed = 30
    forward = 0
    basePos = p.getBasePositionAndOrientation(agent)
    x = basePos[0][0]
    y = basePos[0][1]
    if(index < len(schedule[agent])):
        next = schedule[agent][index]
    else:
        return 0, 0
    # print(basePos[0])
    print(agent, x, y, next)

    Orientation = list(p.getEulerFromQuaternion(basePos[1]))
    goal_direct = math.atan2((next["y"] - y), (next["x"] - x))
    goal_direct_2 = goal_direct
    Orient_2 = Orientation[2]
    if goal_direct < 0 and goal_direct < -math.pi / 3:
        goal_direct = goal_direct + math.pi * 2
    if Orientation[2] < 0 and Orientation[2] < -math.pi/3:
        Orientation[2] = Orientation[2] + math.pi*2

    print(agent, Orientation[2], goal_direct)

    if (abs(Orientation[2] - goal_direct) <= 0.05):
        turn = 0
        direct = True
    else:
        forward = 0
        if(goal_direct_2 < -math.pi/2 and Orient_2 > math.pi/2):
            print("here")
            turn = math.pi - Orient_2 + (goal_direct_2 + math.pi)
        elif(goal_direct_2 > math.pi/2 and Orient_2 < -math.pi/2):
            turn = -(math.pi + Orient_2 + math.pi - goal_direct_2)
        else:
            print("here2")
            turn = (goal_direct_2 - Orient_2)
        print(agent, "turn:", turn)
        direct = False

    print("direct:", direct)
    if (direct is True):
        forward = 1
        turn = 0

    if (checkPosWithBias(basePos[0], [next["x"], next["y"]], 0.4)):
        forward = 0
        turn = 0

    # rightWheelVelocity = (forward + turn) * speed
    # leftWheelVelocity = (forward - turn) * speed
    rightWheelVelocity = (forward) * speed + turn * 20
    leftWheelVelocity = (forward) * speed - turn * 20
    return leftWheelVelocity, rightWheelVelocity


def read_input(yaml_file, env_loaded):
    """
        Read input file, load boundaries, robot and obstacles, set up goals dictionary

        Args:

        yaml_file: input yaml file

        env_loaded: True or false, check if the boundaries, robots and obstacles have been loaded before

        Returns:

        agents: list of boxID
        goals: dictionary of goal position for each robot.
        env_loaded: True
    """
    agents = []
    goals = {}
    with open(yaml_file, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
        if(env_loaded is True):
            for i in param["agents"]:
                goals[i["name"]] = i["goal"]
            return None, goals, True
        for i in param["agents"]:
            startPosition = (i["start"][0], i["start"][1], 0)
            boxId = p.loadURDF("data/turtlebot.urdf", startPosition, startOrientation, globalScaling=1)
            agents.append(boxId)
            goals[boxId] = i["goal"]
        dimensions = param["map"]["dimensions"]
        p.resetDebugVisualizerCamera(cameraDistance=dimensions[0] * 0.9, cameraYaw=0, cameraPitch=-89,
                                     cameraTargetPosition=[dimensions[0] / 2, dimensions[1] / 2, 0])

        createBoundaries(dimensions[0], dimensions[1])
        if env_loaded is False:
            for i in param["map"]["obstacles"]["horizontal"]:
                p.loadURDF("data/rectangle_horizontal.urdf", [i[0], i[1], 0.5])
            for i in param["map"]["obstacles"]["vertical"]:
                p.loadURDF("data/rectangle_vertical.urdf", [i[0], i[1], 0.5])
            if(param["map"]["obstacles"]["top_left_corner"] is not None):
                for i in param["map"]["obstacles"]["top_left_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, math.pi/2]))
            if (param["map"]["obstacles"]["top_right_corner"] is not None):
                for i in param["map"]["obstacles"]["top_right_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5])
            if (param["map"]["obstacles"]["bottom_left_corner"] is not None):
                for i in param["map"]["obstacles"]["bottom_left_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, math.pi]))
            if (param["map"]["obstacles"]["bottom_right_corner"] is not None):
                for i in param["map"]["obstacles"]["bottom_right_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, -math.pi/2]))
            if (param["map"]["obstacles"]["horizontal_half"] is not None):
                for i in param["map"]["obstacles"]["horizontal_half"]:
                    p.loadURDF("data/rectangle_horizontal_half.urdf", [i[0], i[1], 0.5])
            if (param["map"]["obstacles"]["vertical_half"] is not None):
                for i in param["map"]["obstacles"]["vertical_half"]:
                    p.loadURDF("data/rectangle_vertical_half.urdf", [i[0], i[1], 0.5])
    return agents, goals, True

def read_output(output_yaml_file):
    """
        Read file from output.yaml, store path list.

        Args:

        output_yaml_file: output file from cbs.

        Returns:

        schedule: path to goal position for each robot.
    """
    with open(output_yaml_file, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return param["schedule"]

def navigation(agents, goals, schedule):
    """
        Set up loop to publish leftwheel and rightwheel velocity for each robot to reach goal position.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        goals: dictionary with boxID as the key and the corresponding goal positions as values
    """
    index = 0
    while (not check_goal_reached(agents, goals)):
        time.sleep(1. / 240.)
        for i in agents:
            leftWheelVelocity, rightWheelVelocity = set_velocity(i, schedule, index)
            print(i, leftWheelVelocity, rightWheelVelocity)
            p.setJointMotorControl2(i, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=10)
            p.setJointMotorControl2(i, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=10)
        if(check_next_reached(agents, schedule, index)):
            index+=1

def drop_ball(agents):
    """
        Drop ball for each robot at their current positions.

        Args:

        agents: array containing the boxID for each agent
    """
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        ball_pos = [pos[0], pos[1], 2]
        p.loadURDF("data/sphere_small.urdf", ball_pos)

def drop_cube(agents):
    """
        Drop cubes for each robot at their current positions.

        Args:

        agents: array containing the boxID for each agent
    """
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        cube_pos = [pos[0], pos[1], 2]
        p.loadURDF("cube.urdf", cube_pos, globalScaling=0.1)

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)
startOrientation = p.getQuaternionFromEuler([0,0,0])
global env_loaded
env_loaded = False

agents, goals, env_loaded = read_input("scene/room_scene_4_bots_env.yaml", env_loaded)
cbs.main("scene/room_scene_4_bots.yaml", "output.yaml")
schedule = read_output("output.yaml")
navigation(agents, goals, schedule)
time.sleep(1)
drop_cube(agents)
_,goals,env_loaded = read_input("scene/room_scene_4_bots_stage_2.yaml", env_loaded)
cbs.main("scene/room_scene_4_bots_stage_2.yaml", "output.yaml")
schedule = read_output("output.yaml")
navigation(agents, goals, schedule)