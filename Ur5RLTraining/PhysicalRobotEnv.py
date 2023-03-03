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

import socket, struct

import threading

class PhysicalUR5Env(gym.Env):
    def __init__(self, visualize_with_bullet = True):
        self.target = self.newRandomTarget()
        self.action_space = gym.spaces.Box(low = -1., high = 1., shape = (6,))
        self.observation_space = gym.spaces.Box(
            low = np.array([-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-np.inf,-np.inf,-np.inf]), 
            high = np.array([2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,np.inf,np.inf,np.inf]), 
            shape = (9,))
        self.robot = Robot()


        # used for automatically disabling protective stop
        self.socket_port_29999 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_port_29999.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_port_29999.connect((self.robot.host, 29999))

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
    
    def updatePybulletVis(self, joints_pos):

        for i,joint in enumerate(self.controllableJoints):
            p.resetJointState(self.robotID, joint, joints_pos[i])
        

    def step(self, action):
        # print(action)

        jointPos, tcp_pos, protective_stop = self.robot.get_data()
        # print('protective_stop:', protective_stop)
        assert(jointPos is not None and tcp_pos is not None)
        if self.visualize_with_bullet:
            self.updatePybulletVis(jointPos)

        self.robot.set_vel_joint(action[0], action[1], action[2], action[3], action[4], action[5])

        ee_x, ee_y, ee_z = tcp_pos[:3]
        distToTarget = math.sqrt((self.target[0] - ee_x)**2 + (self.target[1] - ee_y)**2 + (self.target[2] - ee_z)**2)
        reward = -distToTarget
        done = False
        if distToTarget < 0.1:
            reward = 1
            done = True
        elif protective_stop:
            reward = -300
            done = True
            pass
        
        # print('{:05.3f}, {:05.3f}, {:05.3f}'.format(*tcp_pos[:3]))
        # self.s.send(string.encode('utf8'))
        

        return tuple(pos % (2 * math.pi) for pos in jointPos) + self.target, reward, done, {}

    def reset(self):


        
        # print('b

        joint_pos, _, protective_stop = self.robot.get_data()
        

        while protective_stop:
            self.socket_port_29999.send('unlock protective stop'.encode('utf8'))
            joint_pos, _, protective_stop = self.robot.get_data()
            print('unlocking protective stop')
            
        


        reset_pos = (0,-90 / 180 * math.pi,0,0,0,0)
        for i in range(10):
            self.robot.move_to_joints(*reset_pos)
            sleep(0.1)

        dist_angle = math.sqrt(sum(tuple((a-b)**2 for a,b in zip(joint_pos, reset_pos))))
        

        while dist_angle > 0.1:
            # print(dist_angle)
            joint_pos, _, protective_stop = self.robot.get_data()
            dist_angle = sum(tuple((a-b)**2 for a,b in zip(joint_pos, reset_pos)))
            # self.robot.move_to_joints(*reset_pos)
            if self.visualize_with_bullet:
                self.updatePybulletVis(self.robot.get_data()[0])
        
        self.target = self.newRandomTarget()
        return tuple(pos % (2 * math.pi) for pos in self.robot.get_data()[0]) + self.target

    def newRandomTarget(self):
        target = (0.7 * random.random() - 0.35, 0.7 * random.random() - 0.35 , 0.3 + 0.4 * random.random())
        print('target:')
        print('{:05.3f}, {:05.3f}, {:05.3f}'.format(target[0],target[1],target[2]))
        # return (0,0,0.4)
        return target


    
if __name__ == '__main__':
    # robot = Robot()
    # while True:
    #     print(robot.get_data())
    
    env = PhysicalUR5Env()
    # sleep(2)
    obs = env.reset()


    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.30 * np.ones(n_actions))
    model = DDPG.load('/home/mara/Desktop/Ur5RLTraining/TrainedFromSimulation.zip', action_noise=action_noise, train_freq=(100, 'step'))
    model.set_env(env)
    callback = CheckpointCallback(10**2, 'RoboticsRealWorld')
    model.learn(total_timesteps=10**12, callback = callback)