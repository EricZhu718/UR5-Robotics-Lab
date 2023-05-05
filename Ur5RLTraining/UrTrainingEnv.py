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





class UR5Env(gym.Env):
    def __init__(self, render = False):
        super().__init__()
        

        self.total_reward = 0
        self.target = self.newRandomTarget()
        self.action_space = gym.spaces.Box(low = -1., high = 1., shape = (6,))
        self.observation_space = gym.spaces.Box(
            low = np.array([-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-2 * math.pi,-1,-1,0]), 
            high = np.array([2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,2 * math.pi,1,1,1]), 
            shape = (9,))
        self.rendering = render
        self.steps = 0

        self.physicsClient = p.connect(p.DIRECT if not render else p.GUI)
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
        print(self.controllableJoints)
        print(len(self.controllableJoints))
        if self.rendering:
            self.slowDebug = p.addUserDebugParameter('slow down', 0.0, 0.02)
        self.reset()

    def step(self, action):
        # print(action)
        self.steps += 1
        assert len(action) == 6
        self.setVelJoints(action)
        for i in range(10):
            p.stepSimulation()
        # print(self.steps) 
        # if self.show:
        if self.rendering:
            sleep(p.readUserDebugParameter(self.slowDebug))
        jointPos, jointVel = self.getJointInfo()
        (ee_x,ee_y,ee_z), (qx,qy,qz,qw) = self.getEndEffectorPos()

        distToTarget = math.sqrt((self.target[0] - ee_x)**2 + (self.target[1] - ee_y)**2 + (self.target[2] - ee_z)**2)

        
        reward = -distToTarget
        done = False
        if distToTarget < 0.1:
            reward = 1
            done = True
        else:
            pass
        # if self.steps >= 600:
        #         done = True
        if self.steps % 10000 == 0:
            print('joint pos:   ', '{:04.4f}, {:04.4f}, {:04.4f}, {:04.4f}, {:04.4f}, {:04.4f}'.format(jointPos[0], jointPos[1], jointPos[2], jointPos[3], jointPos[4], jointPos[5]))
        self.total_reward += reward    
        if done:
            
            
            print('target pos:  ', '{:07.4f}, {:07.4f}, {:07.4f}'.format(self.target[0],self.target[1],self.target[2]))
            print('end pos:     ', '{:07.4f}, {:07.4f}, {:07.4f}'.format(ee_x,ee_y,ee_z))
            print('total steps: ', self.steps)
            print('total reward:', self.total_reward)
            print('------end of episode------\n\n')
            self.steps = 0
            self.total_reward = 0
        

        return tuple(jointPos) + self.target, reward, done, {}

    def reset(self):
        states= [0,-math.pi / 2,0,0,0,0]
        for i,joint in enumerate(self.controllableJoints):
            p.resetJointState(self.robotID, joint, states[i])
        
        self.target = self.newRandomTarget()
            
        self.steps = 0
        print('\n\n-----start of episode-----')
        return tuple(states) + self.target

    def newRandomTarget(self):
        target = (0.7 * random.random() - 0.35, 0.7 * random.random() - 0.35 , 0.3 + 0.4 * random.random())
        # print('target:')
        # print('{:05.3f}, {:05.3f}, {:05.3f}'.format(target[0],target[1],target[2]))
        return target

    def setVelJoints(self, velocities):
        for joint, vel in zip(self.controllableJoints, velocities):
            p.setJointMotorControl2(self.robotID, joint, p.VELOCITY_CONTROL, targetVelocity = vel)
    
    def getJointInfo(self):
        pos = []
        vel = []
        for joint in self.controllableJoints:
            info = p.getJointState(self.robotID, joint)

            pos.append(info[0] % (2 * math.pi))
            vel.append(info[1])
        return (pos, vel)

    def getEndEffectorPos(self):
        info = (p.getLinkState(self.robotID, 7))
        return (info[0], info[1]) # (Coordinates, Quaterions)
    

if __name__ == '__main__':
    env = UR5Env(render = True)
    
    env.reset()
    n_actions = env.action_space.shape[-1]
    print(n_actions)
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.5 * np.ones(n_actions))
    # params = [p.addUserDebugParameter(str(i), -2*math.pi, 2*math.pi) for i in range(6)]
    # while True:
    #     action = tuple(p.readUserDebugParameter(param) for param in params)
    #     obs, reward, done, info = env.step(action)
    #     print(len(obs))
    #     print(tuple(round(ob, 4) for ob in obs[:6]))

    model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1)
    # model = DDPG.load("/home/eric/Desktop/pybullet_ur5_robotiq/MoreWeirdStuff/rl_model_600000_steps")
    model.set_env(env)
    print('here')
    callback = CheckpointCallback(10**5, 'URModelParamsNegRewardz=0.2to0.7')
    model.learn(total_timesteps=10**12, callback = callback)
    model.save("URModelFinal")