import socket, struct
from urx import ursecmon



class Robot:
    def __init__(self, host = "169.254.129.1", port = 30001):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.connect((host, port))
        self.angle = None
        self.tcp_pos = None
        self.parser = ursecmon.ParserUtils()
        self.protective_stop = False
        self.host = host
    
    def get_data(self):
        s = self.s
        data = s.recv(4096)
        while len(data) <= 61:
            data = s.recv(4096)
        #initialise i to keep track of position in packet
        i = 0
        if data:
            try:
                self.protective_stop = self.parser.parse(data)['RobotModeData']['isSecurityStopped']
            except:
                pass
            
            #Print title to screen
            #extract packet length, timestamp and packet type from start of packet and print to screen
            packlen =  (struct.unpack('!i', data[0:4]))[0]
            timestamp = (struct.unpack('!Q', data[10:18]))[0]
            packtype = (struct.unpack('!b', data[4:5]))[0] 

            if packtype == 16:
                #if packet type is Robot State, loop until reached end of packet
                while i+5 < packlen:

                    #extract length and type of message and print if desired
                    msglen = (struct.unpack('!i', data[5+i:9+i]))[0] 
                    msgtype = (struct.unpack('!b', data[9+i:10+i]))[0] 
                    if msgtype == 1:
                        #if message is joint data, create a list to store angles
                        angle = [0]*6
                        j = 0
                        while j < 6:
                            #cycle through joints and extract only current joint angle (double precision)  then print to screen
                            #bytes 10 to 18 contain the j0 angle, each joint's data is 41 bytes long (so we skip j*41 each time)
                            angle[j] = (struct.unpack('!d', data[10+i+(j*41):18+i+(j*41)]))[0]
                            j = j + 1
                        self.angle = angle        

                    elif msgtype == 4:
                        #if message type is cartesian data, extract doubles for 6DOF pos of TCP and print to sc    reen
                        
                        x =  (struct.unpack('!d', data[10+i:18+i]))[0]
                        y =  (struct.unpack('!d', data[18+i:26+i]))[0]
                        z =  (struct.unpack('!d', data[26+i:34+i]))[0]
                        rx =  (struct.unpack('!d', data[34+i:42+i]))[0]
                        ry =  (struct.unpack('!d', data[42+i:50+i]))[0]
                        rz =  (struct.unpack('!d', data[50+i:58+i]))[0]

                        self.tcp_pos = [x,y,z,rx,ry,rz]
                    elif msgtype == 20:
                        #if popup data
                        
                        timestamp = (struct.unpack('!d', data[10+i:18+i]))[0]
                        source = (struct.unpack('!i', data[18+i:19+i]))[0]
                        robotMessageType = (struct.unpack('!i', data[19+i:20+i]))[0]
                        robotMessageCode = (struct.unpack('!i', data[20+i:24+i]))[0]
                        robotMessageArgument = (struct.unpack('!i', data[24+i:28+i]))[0]

                        print(timestamp, source, robotMessageType, robotMessageCode, robotMessageArgument)
                        print('popup data')
                        print(data[5+i:5+msglen])

                    i = i + msglen


            #increment i by the length of the message so move onto next message in packet
                
        if self.angle is None or self.tcp_pos is None:
            return self.get_data()
        return self.angle, self.tcp_pos, self.protective_stop
    
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