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

url='rr+tcp://localhost:58651/?service=robot'
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
q0=[1.18523319,  0.53565687,  0.1785436,  -0.00676761,  0.86593169, -0.36804011]
q1=[1.93510252, 0.53591896, 0.17717764, 0.00334864, 0.86794737, 0.37525218]
q2=[1.93506976, 0.72443561, 0.21769904, 0.00419835, 0.63895034, 0.37408059]
q3=[1.93506803, 0.94206041, 0.182576,   0.00561452, 0.4564512,  0.37243542]

# print(q)
robot.jog_freespace(q0, 0.3*np.ones(num_joints), True)
robot.jog_freespace(q1, 0.3*np.ones(num_joints), True)
robot.jog_freespace(q2, 0.3*np.ones(num_joints), True)
robot.jog_freespace(q3, 0.3*np.ones(num_joints), True)



