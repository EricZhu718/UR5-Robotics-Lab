import socket
import struct
import pygame
from time import sleep


angle = []
tcp_pos = []

def init_pygame_screen():
    successes, failures = pygame.init()
    print("{0} successes and {1} failures".format(successes, failures))


    screen = pygame.display.set_mode((720, 480))
    clock = pygame.time.Clock()
    FPS = 60  # Frames per second.

    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    # RED = (255, 0, 0), GREEN = (0, 255, 0), BLUE = (0, 0, 255).

    rect = pygame.Rect((0, 0), (32, 32))
    image = pygame.Surface((32, 32))
    image.fill(WHITE)  
    screen.fill(BLACK)
    screen.blit(image, rect)
    pygame.display.update()  # Or pygame.display.flip()


def get_data(s):
    global angle, tcp_pos
    data = s.recv(4096)
    #initialise i to keep track of position in packet
    i = 0
    if data:
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
                            
                    # print ('*******')

                elif msgtype == 4:
                    #if message type is cartesian data, extract doubles for 6DOF pos of TCP and print to sc    reen
                    
                    x =  (struct.unpack('!d', data[10+i:18+i]))[0]
                    y =  (struct.unpack('!d', data[18+i:26+i]))[0]
                    z =  (struct.unpack('!d', data[26+i:34+i]))[0]
                    rx =  (struct.unpack('!d', data[34+i:42+i]))[0]
                    ry =  (struct.unpack('!d', data[42+i:50+i]))[0]
                    rz =  (struct.unpack('!d', data[50+i:58+i]))[0]

                    tcp_pos = [x,y,z,rx,ry,rz]

                #increment i by the length of the message so move onto next message in packet
                i = msglen + i
    


if __name__ == '__main__':
    HOST = "192.168.7.24"
    HOST = '169.254.129.1'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))
    
    init_pygame_screen()


    keys_pressed = set()
    while True:
        get_data(s)
        
        # print([round(elm, 3) for elm in angle])
        if len(angle) == 6:
            joint_str = '{:01.2f}, {:01.2f}, {:01.2f}, {:01.2f}, {:01.2f}, {:01.2f}'.format(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5] )
            print(joint_str)
            pos_str = '{:01.2f}, {:01.2f}, {:01.2f}, {:01.2f}, {:01.2f}, {:01.2f}'.format(*tcp_pos)
            print(pos_str)
            print('------')
        # print([round(elm, 3) for elm in tcp_pos])
        # print()

        for event in pygame.event.get():
            if event.type == pygame.KEYUP:
                keys_pressed.remove(event.key)
            elif event.type == pygame.KEYDOWN:
                keys_pressed.add(event.key)
        
        # print(keys_pressed)

        speed = 2
        joint_vels = [0] * 6
        target_pos = [elm for elm in tcp_pos]
        if pygame.K_a in keys_pressed:
            joint_vels[1] += speed 
        elif pygame.K_d in keys_pressed:
            joint_vels[1] -= speed

        elif pygame.K_w in keys_pressed:
            joint_vels[0] += speed 
        elif pygame.K_s in keys_pressed:
            joint_vels[0] -= speed


        elif pygame.K_e in keys_pressed:
            joint_vels[2] += speed 
        elif pygame.K_q in keys_pressed:
            joint_vels[2] -= speed

        elif pygame.K_1 in keys_pressed:
            target_pos[1] = tcp_pos[0] - 1
            print('Hi')


        send_str = "speedl([{}, {}, {}, {}, {}, {}], a=1.0)\n".format(*joint_vels)
        print(send_str)
        # if len(target_pos) == 6:
        #     send_str = "movel([{}, {}, {}, {}, {}, {}], a={})\n".format(*target_pos, 1.0)
        #     print(send_str)

        #     # s.send(send_str.encode('utf8'))
        #     sleep(5)