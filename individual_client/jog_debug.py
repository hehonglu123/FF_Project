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

url='rr+tcp://localhost:22222?service=robot'
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

# q=inv.inv(np.array([-0.6,0.3,0.9])+np.array([0.2,0,0]),R)
q=[ 1.55299516,  1.83282743, -3.02658648,  0.00757938, -0.32599355,  2.89831902]

print(q)
robot.jog_freespace(q, np.ones(num_joints), True)



