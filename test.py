import numpy as np
import sys
from  robot_class import Robot
import time

sys.path.insert(1, 'abr_control')
from abr_control.arms import ur5
from abr_control.controllers.path_planners.inverse_kinematics import InverseKinematics



if __name__ == '__main__':
    robot_config = ur5.Config()
    print(robot_config)
    inv_kin = InverseKinematics(robot_config, max_dx = 0.25)
    print(inv_kin)


    robot_io = Robot()

    joint_angles = None
    tcp_pos = None
    while tcp_pos is None:
        joint_angles, tcp_pos = robot_io.get_data()
    time_steps = 1000
    dt = 0.01
    path_vel, path_pos = (inv_kin.generate_path(
                                np.array(tcp_pos), 
                                np.array([
                                            tcp_pos[0] - 0.1, 
                                            tcp_pos[1],
                                            tcp_pos[2],
                                            tcp_pos[3],
                                            tcp_pos[4],
                                            tcp_pos[5]]), 
                                n_timesteps = time_steps, 
                                dt =dt
                                ))
    print(path_vel[100])
    print(path_pos[100])
    start_time = time.time()
    curr_time = time.time()
    while curr_time - start_time < time_steps * dt:
        
        robot_io.set_vel_tcp(*path_vel[int((curr_time - start_time) * dt)])
        # robot_io.set_vel_tcp(*map(lambda x: 1.0 if x > 1.0 else -1.0 if x < -1.0 else x, path_vel[int((curr_time - start_time) * dt)]))
        
        
        
        curr_time = time.time()
    