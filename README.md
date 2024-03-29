# UR5-Robotics-Lab
Github respository for code related optimizing the performance of a Universal Robotics UR5 Robotic Arm at the University of Maryland.

## Basic Guidelines

### 1. Basic Safety
Safety is your #1 priority :). While the robotic arm has bulid-in fail safes that are designed to protect the user from bodily injury, I generally exercise caution in the following ways:
- Try to stay away from the robot when moving, preferrably behind the desk that's in between the robot and the desktop.
- I slowed down the maximum speed of the UR5 arm through the safety tab on the tablet. If you ever need to speed up the arm for whatever reason, the safety password I believe is either "password", "safety", or "admin". Mara set it, so she should know what it is.
- Enable safety planes. Again, Mara should know more about this, but basically setting saftey planes prevent the robot from reaching past a certain location. The robot shutsdown if that were to encounter a safety plane
- Emergency stop button. Pretty self-explanatory: slam the button and the robot arm will stop immediately
In generally, I wouldn't stress too much about getting injured by the arm. The arm has a very sensitive pressure system with the select purpose of protecting its users. What this means is that if the robot arm feels that it is pushing too hard or that it has hit something with even a little bit of force, the arm instanteously haults. You will never be in a situation in which the robot hits itself so hard that it damages itself. If the robot flies at you, I'm pretty sure it will stop mid-contact so that you're not injured beyond a minor bruise (at least I'm pretty confident that's the case, I haven't actually been hit thankfully :).

### 2. Basics of Connecting via Ethernet Cable
- To communicate with the robot, we connected an eternet cable between the robot and the desktop. If the robot ever stops responding, the first thing to check is if the robot is still connected. Using the ping command in terminal to ping the robot ip address which (unless changed) is ``169.254.129.1``. You should see a response. If that doesn't work, please check that the robot's static IP address is the address you pinged, and that the cable is connected. 
- The desktop has its ip reset to be on a compatable ip address to the robot. If you ever choose to connect a different device, make sure the ip address of the other device is of the form ``169.254.129.xxx`` where xxx cannot be 0 or 255 because those are restricted values.

### 3. Software To Connect to the Robot:
- I would recommend looking through the files ``Ur5RLTrainingFromDesktop/robot_class.py`` and ``Ur5RLTrainingFromDesktop/PhysicalRobotEnvForTraining.py``. The former is the class I wrote that automatically establishes a socket connection to the robot (you shouldn't need to change the address and definitely do not change the port number). The latter is my gym environment used to operate the robot in the real world.
- Notice that with my custom environment, I call update ``self.robot.update()`` everytime I want to retrieve the latest information of my robot. I then access the robot's current poses by calling ``self.robot.joint_angle / self.robot.joint_vel / self.robot.tcp_pos / self.robot.tcp_orientation``

### 4. Protective Stops
- If the robot crashes into itself during training, the robot immediately stops moving, and any command sent by the desktop will not run. You need to go to the tablet and physically reactivate the robot before any other computer commands can be run again.
- There is a way to automatically disable the protective stops through ending an utf8-encoded message through port 29999, but it didn't work when I tried it. I also don't think it's a good idea honestly because automatically disabling protective stops may lead to more crashes and hurt the hardware.
- The software can tell when there's a protective stop by accessing the field ``protective_stop`` in the robot class. What I typically do when I'm training something and there's a protective stop is that I give a large negative reward and then end the episode. I keep the code running, disable the protective stop on from the tablet, and then start a new episode.

### 5. Terminology
- ee_pos refers to end-effector position aka the position at the end of the arm
- joint angles and joint velocities are relatively straight forward

### 6. Inverse Kinematics Solver
- There's an inverse kinematics solver built into the robot that is both faster and accurate. To call it, simply say ``self.robot.movel``
- Alteratively, pybullet has a built-in inverse kinematics solver as well
- Mara and I trained an RL for reach task at one point, but the coordinates didn't line up perfectly when we translated the arm from simulation to the real world. If you want to play with it, the simulation code for the pybullet reach task is ``Ur5RLTrainingFromDesktop/UrTrainingEnvSimulation.py``. I believe my simulation code for the clamp task was lost when my laptop died.
- Additionally, there is a trained version of the reach task in ``Ur5RLTrainingFromDesktop/rl_model_2000000_steps.zip``. Again, this will not work perfectly in the real life robot because the coordinate systems are slightly off.

### 7. Gripper
- I never figured out how to open and close the gripper. I think the reason is that the gripper is it's own seperate device.

### 8. Pybullet
- I know Mara uses Mujoco. I used Pybullet because you're able to set the velocity of the joints whereas I don't think you can in Mujoco, although I didn't look very hard
- The physical robot uses euler angles but pybullet uses quaterions. I used scipy to toggle between the two.

### 9. Other commands
- There are many other commands the robot can receive that I did not code up. This is mainly cause the ones given in robot class are the only ones you should need.
- If you search up URScript, you will see the others
- To send a custom command, call ``robot.send_str("your command here")``

### 10. General pitfalls
- When training in pybullet simulation, make sure the replay buffer is large enough. I found out that the replay buffer was not large enough and started deleting earlier episodes. (This was also because I never capped my max steps in my episodes)
-  When operating remotely, make sure on the tablet that the robot is set to remote on the top right when handling in remote
-  Sometimes the robot just times out so turn the thing on and off if it seems to stop responding
-  The table actually moves because it's on wheels. I put reference tape on the ground to help with making sure the table is always in the same position
-  Make sure the coordinate system in the robot is the same as the coordinate system in the simulation
-  

### Miscellaneous Things:
- Joint angles ranges can be increased in the tablet
- Information is sent and received through a header format given by ``ClientInterfaces_Primary.pdf`` (kinda like TCP protocol but slightly different)
- Note that the specific verison of the UR5 we're using is the UR5e, which has slightly different protocols than the the original UR5
- Port 30001 was used because it included all the information needed
- Port 30002 should not be used because the extra information sent to the computer is enough to crash the parser
- Parser (UrSecmon) was taken from the UrX library, but was created for a different version of the robot, which is why I tend to surround each call in try catch statements
- GoPro code is in testgopro.py
- QR code reader library that I used was opencv's and allowed for detecting QR codes that were not perfectly flat with the camera
- URDF files for the robot can be found in ``Ur5RLTraining/urdf``


https://user-images.githubusercontent.com/49328304/213344607-bc15cc64-4d20-4ee0-96bc-67a44516a979.mp4

