import pybullet as p
import time
import pybullet_data
import yaml
from cbs import cbs
import math
import threading

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

def navigation(agent, goal1, schedule1, goal2, schedule2, target):
    """
        Set velocity for robots to follow the path in the schedule.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        index: index of the current position in the path.

        Returns:

        Leftwheel and rightwheel velocity.
    """
    basePos = p.getBasePositionAndOrientation(agent)
    index = 0
    dis_th = 0.3
    while(not checkPosWithBias(basePos[0], goal1, dis_th)):
        basePos = p.getBasePositionAndOrientation(agent)
        next = [schedule1[index]["x"], schedule1[index]["y"]]
        if(checkPosWithBias(basePos[0], next, dis_th)):
            index = index + 1
        if(index == len(schedule1)):
            p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
            p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
            break
        x = basePos[0][0]
        y = basePos[0][1]
        Orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((schedule1[index]["y"] - y), (schedule1[index]["x"] - x))
        if(Orientation < 0):
            Orientation = Orientation + 2 * math.pi
        if(goal_direction < 0):
            goal_direction = goal_direction + 2 * math.pi

        theta = goal_direction - Orientation

        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi

        current = [x, y]

        distance = math.dist(current, next)

        k1 = 5
        k2 = 20

        # linear = k1 * (distance) * math.cos(theta) + 1.0
        A=20
        # print(agent, "distance", distance)
        # print(agent, "exp", math.exp(k1 * distance))
        # print(agent, "distance", distance, "exp", math.exp(k1 * distance), A*math.exp(k1 * distance))
        linear =min(A*math.exp(k1 * distance), 30.0)
        # print(agent, linear)
        angular = k2 * theta

        rightWheelVelocity = linear + angular
        leftWheelVelocity = linear - angular

        p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=100)
        p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=100)

    basePos = p.getBasePositionAndOrientation(agent)
    index = 0
    index_2 = 0
    while (not checkPosWithBias(basePos[0], goal2, dis_th)):

        if(index_2 < len(target)):
            next_target = target[index_2]
            if (checkPosWithBias(basePos[0], next_target, dis_th)):
                print(agent, next_target)

                p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
                p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
                cube_pos = [next_target[0], next_target[1]+1, 1.5]
                time.sleep(0.68)
                p.loadURDF("data/small_cube.urdf", cube_pos, globalScaling=1)
                index_2 = index_2 + 1

        basePos = p.getBasePositionAndOrientation(agent)
        next = [schedule2[index]["x"], schedule2[index]["y"]]
        if (checkPosWithBias(basePos[0], next, dis_th)):
            index = index + 1
        if (index == len(schedule2)):
            p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
            p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
            break
        x = basePos[0][0]
        y = basePos[0][1]
        Orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((schedule2[index]["y"] - y), (schedule2[index]["x"] - x))
        if (Orientation < 0):
            Orientation = Orientation + 2 * math.pi
        if (goal_direction < 0):
            goal_direction = goal_direction + 2 * math.pi

        theta = goal_direction - Orientation

        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi

        current = [x, y]

        distance = math.dist(current, next)

        k1 = 5
        k2 = 35

        # linear = k1 * (distance) * math.cos(theta) + 1.0
        A = 20
        # print(agent, "distance", distance)
        # print(agent, "exp", math.exp(k1 * distance))
        # print(agent, "distance", distance, "exp", math.exp(k1 * distance), A * math.exp(k1 * distance))
        linear = min(A * math.exp(k1 * distance), 30.0)
        # print(agent, linear)
        angular = k2 * theta

        rightWheelVelocity = linear + angular
        leftWheelVelocity = linear - angular

        p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=100)
        p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=100)



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
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # disable tinyrenderer, software (CPU) renderer, we don't use it here
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        for i in param["agents"]:
            startPosition = (i["start"][0], i["start"][1], 0)
            boxId = p.loadURDF("data/turtlebot.urdf", startPosition, startOrientation, globalScaling=1)
            agents.append(boxId)
            goals[boxId] = i["goal"]
        dimensions = param["map"]["dimensions"]
        p.resetDebugVisualizerCamera(cameraDistance=6.8, cameraYaw=0, cameraPitch=-89.9,
                                     cameraTargetPosition=[7.5, 3.8, 0])

        print(param["map"]["obstacles"].keys())
        if env_loaded is False:
            if (param["map"]["obstacles"]["horizontal"] is not None):
                for i in param["map"]["obstacles"]["horizontal"]:
                    p.loadURDF("data/rectangle_horizontal_3m_0_3m.urdf", [i[0], i[1], 0.5])
            for i in param["map"]["obstacles"]["horizontal10"]:
                p.loadURDF("data/rectangle_horizontal_10m.urdf", [i[0], i[1], 0.5])
            for i in param["map"]["obstacles"]["horizontal15"]:
                p.loadURDF("data/rectangle_horizontal_15m.urdf", [i[0], i[1], 0.5])
            for i in param["map"]["obstacles"]["vertical"]:
                p.loadURDF("data/rectangle_vertical_2m_0_3m.urdf", [i[0], i[1], 0.5])
            for i in param["map"]["obstacles"]["vertical9"]:
                p.loadURDF("data/rectangle_vertical_11m.urdf", [i[0], i[1], 0.5])
            if(param["map"]["obstacles"]["top_left_corner"] is not None):
                for i in param["map"]["obstacles"]["top_left_corner"]:
                    p.loadURDF("data/rectangle_corner_0.3m_wide_reverse.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, math.pi/2]))
            if (param["map"]["obstacles"]["top_right_corner"] is not None):
                for i in param["map"]["obstacles"]["top_right_corner"]:
                    p.loadURDF("data/rectangle_corner_0.3m_wide.urdf", [i[0], i[1], 0.5])
            if (param["map"]["obstacles"]["bottom_left_corner"] is not None):
                for i in param["map"]["obstacles"]["bottom_left_corner"]:
                    p.loadURDF("data/rectangle_corner_0.3m_wide.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, math.pi]))
            if (param["map"]["obstacles"]["bottom_right_corner"] is not None):
                for i in param["map"]["obstacles"]["bottom_right_corner"]:
                    p.loadURDF("data/rectangle_corner_0.3m_wide_reverse.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, -math.pi/2]))
            if (param["map"]["obstacles"]["horizontal_half"] is not None):
                for i in param["map"]["obstacles"]["horizontal_half"]:
                    p.loadURDF("data/rectangle_horizontal_half.urdf", [i[0], i[1], 0.5])
            if (param["map"]["obstacles"]["vertical_half"] is not None):
                for i in param["map"]["obstacles"]["vertical_half"]:
                    p.loadURDF("data/rectangle_vertical_half_0.3m_wide.urdf", [i[0], i[1], 0.5])
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
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

def run(agents, goals1, schedule1, goals2, schedule2, targets):
    """
        Set up loop to publish leftwheel and rightwheel velocity for each robot to reach goal position.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        goals: dictionary with boxID as the key and the corresponding goal positions as values
    """
    threads = []
    for agent in agents:
        t = threading.Thread(target=navigation, args=(agent, goals1[agent], schedule1[agent], goals2[agent], schedule2[agent], targets[agent]))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()


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
        cube_pos = [pos[0], pos[1], 1]
        cubeID = p.loadURDF("data/small_cube.urdf", cube_pos, globalScaling=1)
        jointIndex = -1  # -1 implies the base
        parentFramePosition = [0, 0, 0.45]
        childFramePosition = [0, 0, 0]
        parentFrameOrientation = [0, 0, 0, 1]
        childFrameOrientation = [0, 0, 0, 1]

        fixedJoint = p.createConstraint(
            parentBodyUniqueId=i,
            parentLinkIndex=jointIndex,
            childBodyUniqueId=cubeID,
            childLinkIndex=jointIndex,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=parentFramePosition,
            childFramePosition=childFramePosition,
            parentFrameOrientation=parentFrameOrientation,
            childFrameOrientation=childFrameOrientation
        )

physicsClient = p.connect(p.GUI, options='--width=1920 --height=1080 --mp4=multi_deliver.mp4 --mp4fps=30')
# physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")


p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)
startOrientation = p.getQuaternionFromEuler([0,0,0])
global env_loaded
env_loaded = False
agents, goals1, env_loaded = read_input("scene/2_path_scene_4_bots/2_path_scene_4_bots_env.yaml", env_loaded)
cbs.main("scene/2_path_scene_4_bots/2_path_scene_4_bots.yaml", "output.yaml")
schedule1 = read_output("output.yaml")


_,goals2,env_loaded = read_input("scene/2_path_scene_4_bots/2_path_scene_4_bots_stage_2.yaml", env_loaded)
cbs.main("scene/2_path_scene_4_bots/2_path_scene_4_bots_stage_2.yaml", "output.yaml")
schedule2 = read_output("output.yaml")

targets = {
    3: [ [8,6], [7,6], [6,6], [5,6], [4,6], [3,6], [2,6] ],
    4: [ [11,6], [10,6], [9,6]],
    1: [ [12,2], [11,2], [10,2], [9,2], [8,2], [7,2], [6,2], [5,2], [4,2], [3,2], [2,2] ],
    2: [ [16,2], [15,2], [14,2], [13,2]]
}
# print(goals2)
# time.sleep(1000)
run(agents, goals1, schedule1, goals2, schedule2, targets)
time.sleep(2)