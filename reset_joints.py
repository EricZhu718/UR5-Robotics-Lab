import math
from  robot_class import Robot



if __name__ == '__main__':
    robot_io = Robot(host = '169.254.129.1')

    robot_io.move_to_joints(0,-70 / 180 * math.pi,-40/ 180 * math.pi,0,0,0)