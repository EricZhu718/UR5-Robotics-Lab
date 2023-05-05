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


if __name__ == '__main__':
        render = True
        physicsClient = p.connect(p.DIRECT if not render else p.GUI)
        p.setGravity(0,0,-9.806)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeID = p.loadURDF("plane.urdf")
        robotID = p.loadURDF('urdf/real_arm.urdf', basePosition = [0.0, 0.0, -0.1], baseOrientation = p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True, flags = p.URDF_USE_SELF_COLLISION)
        numJoints = p.getNumJoints(robotID)
        
        def getEndEffectorPos():
            # info = (p.getLinkState(robotID, 6))
            info = (p.getLinkState(robotID, 7))
            return (info[0], info[1]) # (Coordinates, Quaterions)


        def getJointInfo():
            pos = []
            vel = []
            for i in range(7):
                info = p.getJointState(robotID, i)

                pos.append(info[0])
                vel.append(info[1])
            return (pos, vel)
        
        numJoints = p.getNumJoints(robotID)

        controllableJoints = []
        for i in range(numJoints):
            info = p.getJointInfo(robotID, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2] 
            controllable = (jointType != p.JOINT_FIXED)

            if controllable:
                controllableJoints.append(jointID)

        x_in = p.addUserDebugParameter('x', -1.0, 1.0)
        y_in = p.addUserDebugParameter('y', -1.0, 1.0)
        z_in = p.addUserDebugParameter('z', 0, 1.0)

        states= [0,-math.pi / 2,0,0,0,0]
        for i,joint in enumerate(controllableJoints):
            p.resetJointState(robotID, joint, states[i])
        
        while True:
                x = p.readUserDebugParameter(x_in)
                y = p.readUserDebugParameter(y_in)
                z = p.readUserDebugParameter(z_in)
                orn = p.getQuaternionFromEuler([0, -math.pi, 0])
                poses = p.calculateInverseKinematics(robotID, 7, [x,y,z])
                for pose in poses:
                    #  print(round(pose, 4))
                    pass
                # print(poses)
                for i in range(10):
                    p.stepSimulation()
                for joint, pos in zip(controllableJoints, poses):
                    # print(joint)
                    p.setJointMotorControl2(robotID, joint, p.POSITION_CONTROL, targetPosition=pos)
                    # p.setJointMotorControl2(robotID, joint, p.POSITION_CONTROL, targetPosition=0)
# 
                pos, ori = getEndEffectorPos()
                print(round(pos[0], 3), round(pos[1], 3), round(pos[2], 3))