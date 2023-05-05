import pybullet as p
import pybullet_data
from time import sleep
import gym
import math
import stable_baselines3 as sbl3
from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
import numpy as np
from UrTrainingEnv import UR5Env


if __name__ == '__main__':
    # model = DDPG.load("/home/eric/Desktop/pybullet_ur5_robotiq/URModelParamsNegRewardz=0.5Learn=0.001/rl_model_2000000_steps")
    # model = DDPG.load("/home/mara/Desktop/URModelParamsNegRewardz=0.2to0.7/rl_model_4500000_steps.zip")

    # model = DDPG.load('/media/mara/Samsung USB/URModelParams3/' + 'rl_model_3700000_steps')
    # model = DDPG.load('/media/mara/Samsung USB/URModelParams2/' + 'rl_model_4600000_steps')
    model = DDPG.load('/home/mara/Desktop/Ur5RLTraining/TrainedFromSimulation.zip')
    env = UR5Env(render=True)
    obs = env.reset()
    x_in = p.addUserDebugParameter('x', -0.4, 0.4)
    y_in = p.addUserDebugParameter('y', -0.4, 0.4)
    z_in = p.addUserDebugParameter('z', 0.4, 0.8)
    print('got here')
    while True:
        x = p.readUserDebugParameter(x_in)
        y = p.readUserDebugParameter(y_in)
        z = p.readUserDebugParameter(z_in)
        obs = tuple(obs)[:-3] + (x,y,z)
        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        # print(rewards)
        # print(obs[6:])
        (x_pos,y_pos,z_pos), orient = env.getEndEffectorPos()
        # print('{:07.4f}, {:07.4f}, {:07.4f}'.format(x,y,z))
        print(math.sqrt((x_pos-x)**2 + (y_pos-y)**2 + (z_pos-z)**2))
        sleep(0.01)
        if done:
            obs = env.reset()
            pass