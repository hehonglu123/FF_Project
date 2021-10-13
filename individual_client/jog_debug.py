from RobotRaconteur.Client import *
import sys, time, yaml, traceback
import numpy as np
from importlib import import_module
sys.path.append('../toolbox/')
from general_robotics_toolbox import *    




robot_name='abb'


sys.path.append('../toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)

url='rr+tcp://pi_fuse:58651/?service=robot'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode

num_joints=len(robot.robot_info.joint_info)


state_w = robot_sub.SubscribeWire("robot_state")
time.sleep(0.5)
robot_state_wire=state_w.TryGetInValue()

robot_state = robot_state_wire[1]
p=robot_state.kin_chain_tcp[0]['position'] 
R=robot_state.kin_chain_tcp[0]['orientation']
# print(q2R(list(R))) 
# print(p)
print(robot_state_wire[1].joint_position)



# R=R_ee.R_ee_tilt_y(np.pi/4)
q0=[-0.03757475,-0.02503307,0.23709877,0.00419083,1.54763242,-0.06684875]


# print(q)
robot.jog_freespace(q0, 0.1*np.ones(num_joints), True)




