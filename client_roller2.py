#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('toolbox/')
from vel_emulate_sub import EmulatedVelocityControl
from general_robotics_toolbox import *    
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)
sys.path.append('individual_client/')
from fabric_detection import detection

def normalize_dq(q):
	q=0.5*q/(np.linalg.norm(q)) 
	return q   
def ImageToMat(image):
	global image_consts
	if image.image_info.encoding == image_consts["ImageEncoding"]["bgr888"]:
		frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')
	elif image.image_info.encoding == image_consts["ImageEncoding"]["depth_u16"]:
		depth_img =image.data.view(dtype=np.uint16).reshape([image.image_info.height, image.image_info.width], order='C')
		frame2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.1), cv2.COLORMAP_JET)
	else:
		assert False, "Unexpected data type"
	return frame2

current_frame=None
#This function is called when a new pipe packet arrives
def new_frame(pipe_ep):
	global current_frame
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame=ImageToMat(image)

		return
###realsense connect
# url='rr+tcp://localhost:25415?service=Multi_Cam_Service'

# #Startup, connect, and pull out the camera from the objref    
# Multi_Cam_obj=RRN.ConnectService(url)
# image_consts = RRN.GetConstants('com.robotraconteur.image', Multi_Cam_obj)

# #Connect the pipe FrameStream to get the PipeEndpoint p
# rgb_cam=Multi_Cam_obj.get_cameras(0)
# p=rgb_cam.frame_stream.Connect(-1)
# #Set the callback for when a new pipe packet is received to the
# #new_frame function
# p.PacketReceivedEvent+=new_frame
# try:
# 	rgb_cam.start_streaming()
# except: 
# 	traceback.print_exc()
# 	pass


#auto discovery
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()

res=RRN.FindServiceByType("com.robotraconteur.robotics.tool.Tool",
["rr+local","rr+tcp","rrs+tcp"])
url_gripper=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url_gripper=serviceinfo2.ConnectionURL
		break
if url_gripper==None:
	print('gripper service not found')


robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
state_w = robot_sub.SubscribeWire("robot_state")

tool_sub=RRN.SubscribeService(url_gripper)
tool=tool_sub.GetDefaultClientWait(1)
tool_state_w = tool_sub.SubscribeWire("tool_state")


##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
state_flags_enum = robot_const['RobotStateFlags']
halt_mode = robot_const["RobotCommandMode"]["halt"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = position_mode
cmd_w = robot_sub.SubscribeWire("position_command")

#parameter setup
n= len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
robot_def=Robot(H,np.transpose(P),np.zeros(n))

vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)
#enable velocity mode
vel_ctrl.enable_velocity_mode()

orientation=R_ee.R_ee(np.pi/2.)
# fabric_position=np.array([0,0.5,0.122])
fabric_position=np.array([-0.2,0.6,0.0])
place_position=np.array([0.3,0.5,0.02])
roller_position=np.array([-0.67,0.185,0.75])


q_roller=np.array([ 2.88623385,  0.88853247, -1.33318776,  0.01062001, -1.12787189,  0.95884558])
q_roller_down_pick=np.array([ 2.88623385,  0.68042554, -0.6864602,   0.01059369, -1.57213536,  0.94555941])
q_roller_front=np.array([ 2.85294575,  0.18763501, -0.25159374,  0.01220663, -1.50297866,  0.95598282])
q_roller_down=np.array([ 2.88623385,   0.87287708, -0.27821436,  0.01046962, -2.1658814,   0.94555941])

def jog_joint(q):
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.01:
		qdot=1	*(q-vel_ctrl.joint_position())
		vel_ctrl.set_velocity_command(qdot)
	vel_ctrl.set_velocity_command(np.zeros((n,)))
	return

def move_cartesian(vd,factor):
	w=1.
	Kq=.01*np.eye(n)    #small value to make sure positive definite
	KR=np.eye(3)        #gains for position and orientation error

	q_cur=vel_ctrl.joint_position()
	J=robotjacobian(robot_def,q_cur)        #calculate current Jacobian
	Jp=J[3:,:]
	JR=J[:3,:] 
	H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)

	H=(H+np.transpose(H))/2

	robot_pose=fwdkin(robot_def,q_cur.reshape((n,1)))
	R_cur = robot_pose.R
	ER=np.dot(R_cur,np.transpose(orientation))
	k,theta = R2rot(ER)
	k=np.array(k,dtype=float)
	s=np.sin(theta/2)*k         #eR2
	wd=-np.dot(KR,s)  
	f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)
	qdot=factor*normalize_dq(solve_qp(H, f))
	vel_ctrl.set_velocity_command(qdot)

def pick_roller(p):
	# R=R_ee.R_ee_up(np.pi/4)

	# #down
	# q=inv.inv(p+np.array([0.1,0,-0.1]),R)
	# jog_joint(q)
	# q=inv.inv(p+np.array([0,0,-0.1]),R)
	# jog_joint(q)
	# #pick
	# q=inv.inv(p,R)
	# jog_joint(q)
	# # tool.close()
	# time.sleep(0.5)
	# # get out
	# q=inv.inv(p+np.array([0.1,0,0]),R)
	# jog_joint(q)
	# return

	jog_joint(q_roller_front)
	time.sleep(0.2)
	jog_joint(q_roller)
	tool.close()
	time.sleep(0.5)
	jog_joint(q_roller_front)

def place_roller(p):
	# R=R_ee.R_ee_up(np.pi/4)
	# #front
	# q=inv.inv(p+np.array([0.1,0,0]),R)
	# jog_joint(q)
	# #drop
	# q=inv.inv(p,R)
	# jog_joint(q)
	# # tool.open()
	# time.sleep(0.5)
	# # get down
	# q=inv.inv(p+np.array([0,0,-0.1]),R)
	# jog_joint(q)
	# q=inv.inv(p+np.array([0.1,0,-0.1]),R)
	# jog_joint(q)
	# return

	jog_joint(q_roller_front)
	time.sleep(0.2)
	jog_joint(q_roller)
	tool.open()
	time.sleep(0.5)

def pick_fabric_roller(p):
	R=R_ee.R_ee(np.pi/4)
	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0,0.1]),R)
	jog_joint(q)
	qd=inv.inv(p,R)
	move_till_switch(qd)

	# tool.close()
	time.sleep(0.5)
	#move up
	now=time.time()
	while time.time()-now<2:
		move_cartesian(np.array([0,0,0.1]),0.2)



def pick_fabric_soft(p):
	# R=R_ee.R_ee_up(np.pi/4)
	# #down
	# q=inv.inv(p-np.array([0,0,0.1]),R)
	# jog_joint(q)
	# #pick
	# q=inv.inv(p,R)
	# jog_joint(q)
	# # tool.close()
	# time.sleep(0.5)
	# return

	jog_joint(q_roller_down)
	tool.close()
	time.sleep(0.5)

def place_fabric(p):
	R=R_ee.R_ee_tilt_y(np.pi/4)
	#down
	q=inv.inv(p+np.array([-0.6,0,0.1]),R)
	jog_joint(q)
	#drop
	q=inv.inv(p+np.array([-0.5,0,0]),R)
	jog_joint(q)
	q=inv.inv(p,R)
	jog_joint(q)
	# tool.open()
	time.sleep(0.5)
	return
	

def move_till_switch(qd):
	#start performing fabric picking
	robot_state=state_w.TryGetInValue()
	if not robot_state[0]:
		sys.exit("robot not ready")

	
	q_cur=robot_state[1].joint_position
	while np.linalg.norm(q_cur-qd)>0.01:
		robot_state=state_w.TryGetInValue()
		q_cur=robot_state[1].joint_position
		vel_ctrl.set_velocity_command(0.5*(qd-q_cur))
	vel_ctrl.set_velocity_command(np.zeros(n))
	return

	




fabric_height=0.0001

while True:
	#reset tool default state
	tool.open()
	print('pick roller')
	pick_roller(roller_position)
	print('pick fabric1')
	pick_fabric_roller(fabric_position)
	print('place roller')
	place_roller(roller_position)
	print('pick fabric2')
	pick_fabric_soft(roller_position-np.array([0,0,0.2]))
	print('place fabric')
	place_fabric(place_position)