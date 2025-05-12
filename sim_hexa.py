#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import kinematics
from utils import SimpleRobotSimulation
# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4


if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])

# jambe qui bouge arbitrairement
elif args.mode == "moving-leg":
    cross = p.loadURDF("target2/robot.urdf")
    default_pos = [0.0, 0.0, -0.2]
    controls["move_x"] = p.addUserDebugParameter("move_x", -0.4, 0.4, default_pos[0])
    controls["move_y"] = p.addUserDebugParameter("move_y", -0.4, 0.4, default_pos[1])
    controls["move_z"] = p.addUserDebugParameter("move_z", -0.4, 0.0, default_pos[2])
    controls["moving_leg_index"] = p.addUserDebugParameter("moving_leg_index", 0, 5, 0)
# jambe qui bouge arbitrairement


while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"],
            targets["j_thigh_rf"],
            targets["j_tibia_rf"],
            use_rads=True,
        )
        i = -1
        T = []
        for pt in points:
            # Drawing each step of the DK calculation
            i += 1
            T.append(kinematics.rotaton_2D(pt[0], pt[1], pt[2], leg_angle))
            T[-1][0] += leg_center_pos[0]
            T[-1][1] += leg_center_pos[1]
            T[-1][2] += leg_center_pos[2]
            # print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
            )

        # Temp
        sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))
        # sim.setRobotPose(
        #     leg_center_pos, to_pybullet_quaternion(0, 0, 0),
        # )
        state = sim.setJoints(targets)
    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)
    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk0 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )
    # jambe qui bouge arbitrairement
    elif args.mode == "moving-leg":
        x = p.readUserDebugParameter(controls["move_x"])
        y = p.readUserDebugParameter(controls["move_y"])
        z = p.readUserDebugParameter(controls["move_z"])

        alphas = kinematics.computeIK(x, y, z, use_rads=True)
        moving_leg_index = int(p.readUserDebugParameter(controls["moving_leg_index"]))
        if moving_leg_index == 0:
            targets["j_c1_rf"] = alphas[0]
            targets["j_thigh_rf"] = alphas[1]
            targets["j_tibia_rf"] = alphas[2]
        elif moving_leg_index == 1:
            targets["j_c1_lf"] = alphas[0]
            targets["j_thigh_lf"] = alphas[1]
            targets["j_tibia_lf"] = alphas[2]
        elif moving_leg_index == 2:
            targets["j_c1_lr"] = alphas[0]
            targets["j_thigh_lr"] = alphas[1]
            targets["j_tibia_lr"] = alphas[2]
        elif moving_leg_index == 3:
            targets["j_c1_rr"] = alphas[0]
            targets["j_thigh_rr"] = alphas[1]
            targets["j_tibia_rr"] = alphas[2]
        elif moving_leg_index == 4:
            targets["j_c1_lm"] = alphas[0]
            targets["j_thigh_lm"] = alphas[1]
            targets["j_tibia_lm"] = alphas[2]
        elif moving_leg_index == 5:
            targets["j_c1_rm"] = alphas[0]
            targets["j_thigh_rm"] = alphas[1]
            targets["j_tibia_rm"] = alphas[2]

        sim.setJoints(targets)

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle))
        # jambe qui bouge arbitrairement
        
    elif args.mode == "marche":
        robot = SimpleRobotSimulation(sim)
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
                thetas = kinematics.triangle(0, -0.05, 0.03, 0.08, sim.t, leg_id=l)
                for m in range(0,3):
                    robot.legs[l][m].goal_position = thetas[m]

            for l in index_patte2:
                thetas = kinematics.triangle(0, -0.05, 0.03, 0.08, sim.t+1, leg_id= l)
                for m in range(0,3):
                    robot.legs[l][m].goal_position = thetas[m]

    # fin marche robot
    # testIKOriented(sim, robot)
    sim.tick()

