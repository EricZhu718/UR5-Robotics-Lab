


from  robot_class import Robot
import time
from time import sleep



if __name__ == '__main__':
    robot_io = Robot()
    robot_io.get_data()[1]
    robot_io.send_str('movel(p[-0.11120325378743226, -0.1898752646139312, 0.9505440933521707, 1.0503650829624886, 1.3960240014706027, -1.6396699585797114],a=0.01,v=0.05, t = 1.0)\n')
    print('Sent')
    while True:
        print(robot_io.get_data()[1])
