import pybullet as p
import pybullet_data
from time import sleep
import gym
import math
import stable_baselines3 as sbl3
from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
import numpy as np
import random

from stable_baselines3.common.callbacks import CheckpointCallback


from robot_class import Robot



if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)
    p.setGravity(0,0,-9.806)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeID = p.loadURDF("plane.urdf")
    # robotID = p.loadURDF('/home/eric/Desktop/pybullet_ur5_robotiq/MoreWeirdStuff/customURDF.urdf', basePosition = [0.0, 0.0, 0.0], baseOrientation = p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True, flags = p.URDF_USE_SELF_COLLISION)
    robotID = p.loadURDF('urdf/real_arm.urdf', 
                            basePosition = [0.0, 0.0, -0.1], 
                            baseOrientation = p.getQuaternionFromEuler([0, 0, 0]), 
                            useFixedBase=True, 
                            flags = p.URDF_USE_SELF_COLLISION)
    numJoints = p.getNumJoints(robotID)
    controllableJoints = []
    for i in range(numJoints):
        info = p.getJointInfo(robotID, i)
        jointID = info[0]
        jointName = info[1].decode("utf-8")
        jointType = info[2] 
        controllable = (jointType == p.JOINT_REVOLUTE)

        if controllable:
            controllableJoints.append(jointID)
    
    
    
    robot = Robot()
    reset_pos = (0,-90 / 180 * math.pi,0,0,0,0)
    for i,joint in enumerate(controllableJoints):
            p.resetJointState(robotID, joint, reset_pos[i])
    robot.move_to_joints(*reset_pos)
    sleep(5)


    p.setJointMotorControl2(robotID, controllableJoints[4], p.VELOCITY_CONTROL, targetVelocity = 1)
    while True:
        p.stepSimulation()
        robot.set_vel_joint(0, 0, 0, 0, 1, 0)
        sleep(1/240)