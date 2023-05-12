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

from stable_baselines3.common.callbacks import *

from robot_class import Robot

import socket, struct

import threading

class PhysicalUR5Env(gym.Env):
    def __init__(self, visualize_with_bullet = True):
        self.target = self.newRandomTarget()
        self.action_space = gym.spaces.Box(low = -1., high = 1., shape = (6,))
        self.observation_space = gym.spaces.Dict({
            'observation': 
                gym.spaces.Box( # first 6 are joint positions, next 6 are joint velocities
                        low = np.array([-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi, -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf]), 
                        high = np.array([2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi, np.inf,np.inf,np.inf,np.inf,np.inf,np.inf]), 
                        shape = (12,)
                ),
            'achieved_goal':
                gym.spaces.Box( # first 6 are joint positions, next 6 are joint velocities
                    low = np.array([-np.inf, -np.inf, -np.inf]), 
                    high = np.array([np.inf, np.inf, np.inf]), 
                    shape = (3,)
                ),
            'desired_goal':
                gym.spaces.Box( # first 6 are joint positions, next 6 are joint velocities
                    low = np.array([-np.inf, -np.inf, -np.inf]), 
                    high = np.array([np.inf, np.inf, np.inf]), 
                    shape = (3,)
                )}
            )
        
        self.robot = Robot()
        self.steps = 0

        self.visualize_with_bullet = visualize_with_bullet

        if visualize_with_bullet:
            self.physicsClient = p.connect(p.GUI)
            p.setGravity(0,0,-9.806)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.planeID = p.loadURDF("plane.urdf")
            # self.robotID = p.loadURDF('/home/eric/Desktop/pybullet_ur5_robotiq/MoreWeirdStuff/customURDF.urdf', basePosition = [0.0, 0.0, 0.0], baseOrientation = p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True, flags = p.URDF_USE_SELF_COLLISION)
            self.robotID = p.loadURDF('urdf/real_arm.urdf', 
                                    basePosition = [0.0, 0.0, -0.1], 
                                    baseOrientation = p.getQuaternionFromEuler([0, 0, 0]), 
                                    useFixedBase=True, 
                                    flags = p.URDF_USE_SELF_COLLISION)
            self.numJoints = p.getNumJoints(self.robotID)
            self.controllableJoints = []
            for i in range(self.numJoints):
                info = p.getJointInfo(self.robotID, i)
                jointID = info[0]
                jointName = info[1].decode("utf-8")
                jointType = info[2] 
                controllable = (jointType == p.JOINT_REVOLUTE)

                if controllable:
                    self.controllableJoints.append(jointID)
            self.visualObj = p.createMultiBody(
                            baseMass = 0,
                            baseVisualShapeIndex = p.createVisualShape(shapeType = p.GEOM_SPHERE, 
                                                            radius = 0.03, 
                                                            rgbaColor = [1,0,1,1], 
                                                            visualFramePosition = [0,0,0]),
                            basePosition = [0,0,0]
                        )
            
    
    def updatePybulletVis(self, joints_pos):

        for i,joint in enumerate(self.controllableJoints):
            p.resetJointState(self.robotID, joint, joints_pos[i])
        # print('\n', p.getLinkState(self.robotID, 7)[0])
        # print(self.robot.tcp_pos, '\n')
        print()
        print(p.getLinkState(self.robotID, 7)[0][0] - self.robot.tcp_pos[0])
        print(p.getLinkState(self.robotID, 7)[0][1] - self.robot.tcp_pos[1])
        print(p.getLinkState(self.robotID, 7)[0][2] - self.robot.tcp_pos[2])
        print()
    def step(self, action):
        # print(action)
        self.steps += 1
        self.robot.update()
        joint_pos = self.robot.joint_angle
        joint_vel = self.robot.joint_vel
        tcp_pos = self.robot.tcp_pos
        protective_stop = self.robot.protective_stop

        # print('protective_stop:', protective_stop)
        if self.visualize_with_bullet:
            self.updatePybulletVis(joint_pos)

        self.robot.set_vel_joint(action[0], action[1], action[2], action[3], action[4], action[5])

        ee_x, ee_y, ee_z = tcp_pos
        distToTarget = math.sqrt((self.target[0] - ee_x)**2 + (self.target[1] - ee_y)**2 + (self.target[2] - ee_z)**2)
        reward = -distToTarget
        done = False
        if distToTarget < 0.1:
            reward = 300
            done = True
        elif protective_stop:
            reward = -100
            done = True
            pass
        elif self.steps > 300:
            done = True
        

        return {'observation':np.concatenate((np.array(joint_pos),np.array(joint_vel))),
                'desired_goal': self.target,
                'achieved_goal':(ee_x,ee_y,ee_z)
                }, reward, done, {}

    def reset(self):
        self.steps = 0

        
        # print('b
        self.robot.update()
        joint_pos= self.robot.joint_angle
        protective_stop = self.robot.protective_stop 

        while protective_stop:
            self.robot.update()
            joint_pos= self.robot.joint_angle
            protective_stop = self.robot.protective_stop 
            print('unlock protective stop')
            
        


        reset_pos = (0,-90 / 180 * math.pi,0,0,0,0)
        for i in range(10):
            self.robot.update()
            joint_pos= self.robot.joint_angle
            protective_stop = self.robot.protective_stop 
            if not self.robot.program_running:
                self.robot.move_to_joints(*reset_pos)
            sleep(0.1)

        dist_angle = math.sqrt(sum(tuple((a-b)**2 for a,b in zip(joint_pos, reset_pos))))
        

        while dist_angle > 0.1:
            self.robot.update()
            joint_pos= self.robot.joint_angle
            protective_stop = self.robot.protective_stop 
            if not self.robot.program_running:
                self.robot.move_to_joints(*reset_pos)
            dist_angle = sum(tuple((a-b)**2 for a,b in zip(joint_pos, reset_pos)))
            # self.robot.move_to_joints(*reset_pos)
            if self.visualize_with_bullet:
                self.updatePybulletVis(joint_pos)
        
        self.target = self.newRandomTarget()
        joint_vel = self.robot.joint_vel
        ee_x, ee_y, ee_z = self.robot.tcp_pos
        p.resetBasePositionAndOrientation(self.visualObj, self.target, [0,0,0,1])
        return {'observation':np.concatenate((np.array(joint_pos),np.array(joint_vel))),
                'desired_goal': self.target,
                'achieved_goal':(ee_x,ee_y,ee_z)
                }

    def newRandomTarget(self):
        radius = 0.6 * random.random() + 0.2
        angle = 2*math.pi * random.random()

        target = (radius * math.cos(angle), radius * math.sin(angle), 0.1 + 0.3 * random.random())
        # print('target:')
        # print('{:05.3f}, {:05.3f}, {:05.3f}'.format(target[0],target[1],target[2]))
        return target


    
if __name__ == '__main__':
    env = PhysicalUR5Env()
    # sleep(2)
    obs = env.reset()


    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.30 * np.ones(n_actions))
    model = DDPG.load('/home/mara/Desktop/Ur5RLTraining/rl_model_2000000_steps.zip', action_noise=action_noise) #, train_freq=(100, 'step'))
    model.set_env(env)
    callbackParams = CheckpointCallback(10**2, 'RoboticsRealWorld1')
    callbackBuffer = CheckpointCallback(10**3, 'RoboticsRealWorld1', save_replay_buffer=True)
    callbacklist = CallbackList([callbackParams, callbackBuffer])
    model.learn(total_timesteps=10**12, callback = callbacklist)