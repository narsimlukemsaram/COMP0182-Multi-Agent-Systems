import pybullet as p
import pybullet_data
import math


def check_pos(Pos, goal, bias):
    """
        Check if pos is at goal with bias

        Args:

        Pos: Position to be checked, [x, y]

        goal: goal position, [x, y]

        bias: bias allowed

        Returns:

        True if pos is at goal, False otherwise
    """
    if goal[0] + bias > Pos[0] > goal[0] - bias and goal[1] + bias > Pos[1] > goal[1] - bias:
        return True
    else:
        return False


def goto(agent, goal_x, goal_y):
    dis_th = 0.1
    basePos = p.getBasePositionAndOrientation(agent)
    current_x = basePos[0][0]
    current_y = basePos[0][1]
    pos = [current_x, current_y]
    goal = [goal_x, goal_y]
    while not check_pos(pos, goal, dis_th):
        basePos = p.getBasePositionAndOrientation(agent)
        current_x = basePos[0][0]
        current_y = basePos[0][1]
        pos = [current_x, current_y]
        current_orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((goal_y - current_y), (goal_x - current_x))
        if current_orientation < 0:
            current_orientation = current_orientation + 2 * math.pi
        if goal_direction < 0:
            goal_direction = goal_direction + 2 * math.pi

        theta = goal_direction - current_orientation
        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi

        k_linear = 10
        k_angular = 30
        linear = k_linear * math.cos(theta)
        angular = k_angular * theta

        rightWheelVelocity = linear + angular
        leftWheelVelocity = linear - angular

        p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=10)
        p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=10)


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)

startPosition = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

boxId = p.loadURDF("data/turtlebot.urdf", startPosition, startOrientation, globalScaling=1)
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-89.9,
                             cameraTargetPosition=[0, 0, 0])

goto(boxId, 1, 1)
