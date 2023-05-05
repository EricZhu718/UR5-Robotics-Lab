import gym
import numpy
from  robot_class import Robot

class RealWorldUR5(gym.env):
    def __init__(self, ip = '192.168.7.24'):
        self.connection = Robot(ip = ip)
        self.action_space = gym.box



if __name__ == '__main__':

    pass