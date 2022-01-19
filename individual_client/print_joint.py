#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml, argparse
from importlib import import_module
sys.path.append('../toolbox')
from abb_def import *

fusing_laptop='192.168.51.116'
robosewclient='192.168.51.61'
pi_fuse='192.168.51.25'
my_laptop='192.168.51.181'


# #Accept the names of the webcams and the nodename from command line
# parser = argparse.ArgumentParser(description="RR plug and play client")
# parser.add_argument("--robot-name",type=str,default='abb',help="List of camera names separated with commas")
# args, _ = parser.parse_known_args()
# robot_name=args.robot_name

# sys.path.append('../toolbox')
# from general_robotics_toolbox import Robot, q2R
# from abb_ik import *
# #auto discovery
# time.sleep(2)
# res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
# ["rr+local","rr+tcp","rrs+tcp"])
# url=None
# for serviceinfo2 in res:
# 	if robot_name in serviceinfo2.NodeName:
# 		url=serviceinfo2.ConnectionURL
# 		break
# if url==None:
# 	print('service not found')
# 	sys.exit()

####################Start Service and robot setup
url='rr+tcp://'+robosewclient+':58651?service=robot'


robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
state_flags_enum = robot_const['RobotStateFlags']

state_w = robot_sub.SubscribeWire("robot_state")


print(robot.robot_info.device_info.device.name)

time.sleep(1.5)
robot_state_wire=state_w.TryGetInValue()
print("wire value set: ",robot_state_wire[0])
robot_state = robot_state_wire[1]
for flag_name, flag_code in state_flags_enum.items():
	if flag_code & robot_state.robot_state_flags != 0:
		print(flag_name,flag_code)
		#ready262144
print("robot_joints: ", robot_state.joint_position)
print("kin_chain_tcp(fwd): ", fwd(robot_state.joint_position).p)
print("kin_chain_tcp_R(fwd): ",fwd(robot_state.joint_position).R)
print("joint torque: ", robot_state.joint_effort)


position=robot_state.kin_chain_tcp[0]['position'] 
orientation=robot_state.kin_chain_tcp[0]['orientation'] 
print(position)
print(q2R(list(orientation)))

