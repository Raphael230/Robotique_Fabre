import utils
from utils import SimpleRobotSimulation
from onshape_to_robot.simulation import Simulation
import kinematics
import time
import math
import pybullet as p
from scipy.spatial.transform import Rotation

def testIKOriented(sim, robot):
    val= 0.05* math.sin(sim.t)
    for i in range(6):
        leg= robot.legs[i+1]
        thetas= kinematics.computeIKOriented(val, 0, 0, i+1)
        for m in range(0,3):
            leg[m].goal_position = thetas[m]

def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat

robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0, degrees=True))
robot = SimpleRobotSimulation(sim)
robot.init()
print(robot)
# robot.legs[1,legs["j_c1_rf"]]
# robot- SimpleRobotSimulation
# robot.legs- dictionnaire contenant des pattes
# robot.legs[1]-liste des trois simplemotor de la patte1
# robot.legs[1][0] premier moteur de la première patte du robot

robot.legs[1][0].goal_position= math.pi/2
LEG_ANGLES = [
        -math.pi / 4,
        math.pi / 4,
        math.pi / 2,
        3 * math.pi / 4,
        -3 * math.pi / 4,
        -math.pi / 2,
    ]
LEG_CENTER_POS = [
        (0.1248, -0.06164, 0.001116),
        (0.1248, 0.06164, 0.001116),
        (0, 0.1034, 0.001116),
        (-0.1248, 0.06164, 0.001116),
        (-0.1248, -0.06164, 0.001116),
        (0, -0.1034, 0.001116),
    ]

while True:
    robot.tick_read_and_write()
    # print("salut")
    # time.sleep(1.0)
    # print(robot)
    val=10*math.sin(time.time())*math.pi/180
    # respiration robot
    # for m in robot.motors(): 
    #     m.goal_position=val
    # sim.tick()
    #fin respiration

    # marche du robot
    index_patte1 = [1,3,5]
    index_patte2 = [2,4,6]
    keys = p.getKeyboardEvents()
    
    for l in index_patte1:
        thetas = kinematics.triangle(0, -0.01, 0.06, 0.2, sim.t*8, leg_id=l)
        for m in range(0,3):
            robot.legs[l][m].goal_position = thetas[m]

    for l in index_patte2:
        thetas = kinematics.triangle(0, -0.01, 0.06, 0.2, sim.t*8+1, leg_id= l)
        for m in range(0,3):
            robot.legs[l][m].goal_position = thetas[m]

    # fin marche robot
    # testIKOriented(sim, robot)

        sim.tick()
 


