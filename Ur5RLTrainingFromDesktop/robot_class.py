import socket, struct
from urx import ursecmon
import math


class Robot:
    def __init__(self, host = "169.254.129.1", port = 30001):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.connect((host, port))
        
        
        self.joint_angle = [0] * 6
        self.joint_vel = [0] * 6
        self.tcp_pos = [0] * 3
        self.tcp_orientation = [0] * 3
        self.protective_stop = False
        self.program_running = False
        
        
        self.parser = ursecmon.ParserUtils()
        self.host = host
    
    def update(self):

        s = self.s
        data = s.recv(4096)
        try:


            # parser occassionally crashes
            parsed = self.parser.parse(data)

            
            try:
                # will not always have jointData in parsed data
                for i in range(6):
                    self.joint_angle[i] = parsed['JointData']['q_actual'+str(i)]
            except:
                pass
            
            try: 
                # will not always have jointData in parsed data
                for i in range(6):
                    self.joint_vel[i] = parsed['JointData']['qd_actual'+str(i)]
            except:
                pass


            try:
                self.protective_stop = parsed['RobotModeData']['isSecurityStopped']
            except:
                pass

            try:
                self.program_running = parsed['RobotModeData']['isProgramRunning']
            except:
                pass
            
            try:
                self.tcp_pos[0] = parsed['CartesianInfo']['X']
                self.tcp_pos[1] = parsed['CartesianInfo']['Y']
                self.tcp_pos[2] = parsed['CartesianInfo']['Z']
            except:
                pass

            try:
                self.tcp_orientation[0] = parsed['CartesianInfo']['Rx']
                self.tcp_orientation[1] = parsed['CartesianInfo']['Ry']
                self.tcp_orientation[2] = parsed['CartesianInfo']['Rz']
            except:
                pass


            

                
            # if 'RobotModeData' in parsed.keys():
            #     if 'isSecurityStopped' in parsed['RobotModeData'].keys():
            #         self.protective_stop = parsed['RobotModeData']['isSecurityStopped']
            #     if 'isProgramRunning' in parsed['RobotModeData'].keys():
            #         self.program_running = parsed['RobotModeData']['isProgramRunning']
            
            # print('\n', parsed['JointData'].keys(), '\n')
            # print('\n', [180 / math.pi * angle for angle in self.joint_angle],'\n')
            # print(parsed['CartesianInfo'].keys())

            # print('\n', [val for val in self.tcp_pos], '\n')
            # print('\n', [val for val in self.tcp_orientation], '\n')


        except:
            pass

    # def get_data(self):
    #     s = self.s
    #     data = s.recv(4096)
    #     while len(data) <= 61:
    #         data = s.recv(4096)
    #     #initialise i to keep track of position in packet
    #     i = 0
    #     if data:
    #         try:
    #             parsed = self.parser.parse(data)
    #             if 'RobotModeData' in parsed.keys():
    #                 if 'isSecurityStopped' in parsed['RobotModeData'].keys():
    #                     self.protective_stop = parsed['RobotModeData']['isSecurityStopped']
    #                 if 'isProgramRunning' in parsed['RobotModeData'].keys():
    #                     self.program_running = parsed['RobotModeData']['isProgramRunning']
    #             print(parsed.keys())
    #         except:
    #             pass
    #     if self.angle is None or self.tcp_pos is None:
    #         return self.get_data()
    #     return self.angle, self.tcp_pos, self.protective_stop
    
    def send_str(self,string):
        if self.s is None:
            raise Exception("Error: socket not initialized")
        self.s.send(string.encode('utf8'))
    
    
    def set_vel_tcp(self, v_x,v_y,v_z,vel_angle_1,vel_angle_2,vel_angle_3, a = 1.0):
        self.send_str("speedl([{}, {}, {}, {}, {}, {}], a={})\n".format(v_x,v_y,v_z,vel_angle_1,vel_angle_2,vel_angle_3, a))

    def set_vel_joint(self, j1,j2,j3,j4,j5,j6, a = 1.0):
        self.send_str("speedj([{}, {}, {}, {}, {}, {}], a={})\n".format(j1,j2,j3,j4,j5,j6, a))

    def move_to_tcp_pose(self, x,y,z,angle1,angle2,angle3, a = 1.0):
        self.send_str("movel([{}, {}, {}, {}, {}, {}], a={})\n".format(x,y,z,angle1,angle2,angle3, a))
    
    def move_to_joints(self, j1,j2,j3,j4,j5,j6, a = 1.0):
        self.send_str("movej([{}, {}, {}, {}, {}, {}], a={})\n".format(j1,j2,j3,j4,j5,j6, a))







if __name__ == '__main__':
    myRobot = Robot()

    while True:
        myRobot.update()