#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback
from multiprocessing import Process
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('toolbox/')
from vel_emulate_sub import EmulatedVelocityControl
from general_robotics_toolbox import *    
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)


def normalize_dq(q):
	q=0.5*q/(np.linalg.norm(q)) 
	return q   


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
fabric_position=np.array([-0,0.65,0.])
place_position=np.array([0.0,0.65,0.])

def jog_joint(q):
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.1:
		qdot=1*(q-vel_ctrl.joint_position())
		qdot[:-2]=np.array([x if np.abs(x)>0.1 else 0.1*np.sign(x) for x in qdot])[:-2]
		vel_ctrl.set_velocity_command(qdot)
	vel_ctrl.set_velocity_command(np.zeros((n,)))
	return

def move_cartesian(vd):
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
	qdot=0.05*normalize_dq(solve_qp(H, f))
	vel_ctrl.set_velocity_command(qdot)

def grip(idx):	#idx=1 or 2 first or second gripper
	print('here1')
	try:
		tool.setf_param('roll'+str(idx),RR.VarValue(True,'bool'))
	except:
		traceback.print_exc()
	print('here2')
	time.sleep(0.5)
	while not tool_state_w.InValue.sensor[2*(idx-1)]:
		time.sleep(0.001)

	tool.setf_param('roll'+str(idx),RR.VarValue(False,'bool'))
	tool.setf_param('grip'+str(idx),RR.VarValue(True,'bool'))

def move_till_switch(qd):
	#start performing fabric picking
	robot_state=state_w.TryGetInValue()
	if not robot_state[0]:
		sys.exit("robot not ready")
	
	q_cur=robot_state[1].joint_position
	while np.linalg.norm(q_cur-qd)>0.02:
		robot_state=state_w.TryGetInValue()
		q_cur=robot_state[1].joint_position
		tool_state=tool_state_w.TryGetInValue()	
		if not (robot_state[0] and tool_state[0]):
			sys.exit("robot/sensor not ready")
		vel_ctrl.set_velocity_command(0.1*(qd-q_cur))
		if tool_state[1].sensor[1] or tool_state[1].sensor[3]:
			vel_ctrl.set_velocity_command(np.zeros(n))
			return
	while not (tool_state[1].sensor[1] or tool_state[1].sensor[3]):
		move_cartesian(robot,[0,0,-0.05])
		if tool_state[1].sensor[1] and tool_state[1].sensor[3]:
			vel_ctrl.set_velocity_command(np.zeros(n))
			return
def pick(fabric_position,tool):
	#start joggging to initial pose
	q=inv.inv(fabric_position+np.array([0,0,0.2]),orientation)
	jog_joint(q)
	qd=inv.inv(fabric_position,orientation)
	move_till_switch(qd)

	p1 = Process(target=grip,args=(1,))
	p1.start()
	p2 = Process(target=grip,args=(2,))
	p2.start()
	p1.join()
	p2.join()

	#move up
	jog_joint(q)
	
def place(place_position,tool):
	#start joggging to initial pose
	q=inv.inv(place_position+np.array([0,0,0.2]),orientation)
	jog_joint(q)
	qd=inv.inv(place_position,orientation)
	move_till_switch(qd)
	tool.open()


while True:
	#reset tool default state
	tool.setf_param('roll1',RR.VarValue(False,'bool'))
	tool.setf_param('roll2',RR.VarValue(False,'bool'))
	tool.open()
	time.sleep(0.5)
	pick(fabric_position,tool)
	# place(place_position,tool)

