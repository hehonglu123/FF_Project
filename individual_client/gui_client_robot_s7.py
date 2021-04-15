#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback
from tkinter import *
from tkinter import messagebox
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
sys.path.append('../toolbox')
from vel_emulate_sub import EmulatedVelocityControl

sys.path.append('../toolbox')
from general_robotics_toolbox import *    


def normalize_dq(q):
	q[:-1]=0.5*q[:-1]/(np.linalg.norm(q[:-1])) 
	return q   


#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()
robot_name=args.robot_name


#load eef orientatin
sys.path.append('../toolbox')
R_ee = import_module('R_'+robot_name)

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


#connect
try:
	tool_sub=RRN.SubscribeService(url_gripper)
	tool=tool_sub.GetDefaultClientWait(1)
	tool_state_w = tool_sub.SubscribeWire("tool_state")
	# tool=RRN.ConnectService(url_gripper)
	# tool_state_w=tool.tool_state.Connect()
except:
	traceback.print_exc()
	pass
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
state_w = robot_sub.SubscribeWire("robot_state")
#get params
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))



##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
state_flags_enum = robot_const['RobotStateFlags']
halt_mode = robot_const["RobotCommandMode"]["halt"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = position_mode
cmd_w = robot_sub.SubscribeWire("position_command")

vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)
#enable velocity mode
vel_ctrl.enable_velocity_mode()

#parameter setup
n= len(robot.robot_info.joint_info)

top=Tk()
top.title(robot_name)
jobid = None
def gripper_ctrl(tool):

	if gripper.config('relief')[-1] == 'sunken':
		tool.open()
		gripper.config(relief="raised")
		gripper.configure(bg='red')
		gripper.configure(text='gripper off')

	else:
		tool.close()
		gripper.config(relief="sunken")
		gripper.configure(bg='green')
		gripper.configure(text='gripper on')
	return

def roller_ctrl(tool):

	if roller.config('relief')[-1] == 'sunken':
		tool.setf_param('roll1',RR.VarValue(False,'bool'))
		roller.config(relief="raised")
		roller.configure(bg='red')
		roller.configure(text='roller off')

	else:
		tool.setf_param('roll1',RR.VarValue(True,'bool'))
		roller.config(relief="sunken")
		roller.configure(bg='green')
		roller.configure(text='roller on')
	return

def rotate(n,vd):
	global jobid
	try:
		qdot=np.array([0. for i in range(n)])
		qdot[-1]=vd
		vel_ctrl.set_velocity_command(qdot)
		jobid = top.after(10, lambda: rotate(n, vd))
	except:
		traceback.print_exc()
	return

def move(n, robot_def,vel_ctrl,vd):
	global jobid
	try:
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
		ER=np.dot(R_cur,np.transpose(R_ee.R_ee(0)))
		k,theta = R2rot(ER)
		k=np.array(k,dtype=float)
		s=np.sin(theta/2)*k         #eR2
		wd=-np.dot(KR,s)  
		f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)
		qdot=0.5*normalize_dq(solve_qp(H, f))
		qdot[-1]=0
		vel_ctrl.set_velocity_command(qdot)

		jobid = top.after(10, lambda: move(n, robot_def,vel_ctrl,vd))
	except:
		traceback.print_exc()
	return
def stop(n,vel_ctrl):
	global jobid
	top.after_cancel(jobid)
	vel_ctrl.set_velocity_command(np.zeros((n,)))
	return

##RR part
def update_label():
	robot_state=state_w.TryGetInValue()
	flags_text = "Robot State Flags:\n\n"
	if robot_state[0]:
		for flag_name, flag_code in state_flags_enum.items():
			if flag_code & robot_state[1].robot_state_flags != 0:
				flags_text += flag_name + "\n"
	else:
		flags_text += 'service not running'
		
	joint_text = "Robot Joint Positions:\n\n"
	for j in robot_state[1].joint_position:
		joint_text += "%.2f\n" % np.rad2deg(j)

	tool_state=tool_state_w.TryGetInValue()
	flags_text2 = "Tool State Flags:\n\n"
	if tool_state[0]:
		flags_text2 += 'sensor: '
		if tool_state[1].sensor[0] != 0:
			flags_text2 += 'blocked' + "\n"
		else:
			flags_text2 += 'unblocked' + "\n"
		flags_text2 += 'switch: '
		if tool_state[1].sensor[1] != 0:
			flags_text2 += 'pushed' + "\n"
		else:
			flags_text2 += 'released' + "\n"



	label.config(text = flags_text + "\n\n" + joint_text + "\n\n" + flags_text2)

	label.after(250, update_label)

top.title = "Robot State"

label = Label(top, fg = "black", justify=LEFT)
label.pack()
label.after(250,update_label)


left=Button(top,text='left')
right=Button(top,text='right')
forward=Button(top,text='forward')
backward=Button(top,text='backward')
up=Button(top,text='up')
down=Button(top,text='down')
r_cl=Button(top,text='r_cl')
r_ct=Button(top,text='r_ct')

gripper=Button(top,text='gripper off',command=lambda: gripper_ctrl(tool),bg='red')
roller=Button(top,text='roller off',command=lambda: roller_ctrl(tool),bg='red')

left.bind('<ButtonPress-1>', lambda event: move(num_joints,robot_def,vel_ctrl,[0,.1,0]))
right.bind('<ButtonPress-1>', lambda event: move(num_joints,robot_def,vel_ctrl,[0,-.1,0]))
forward.bind('<ButtonPress-1>', lambda event: move(num_joints,robot_def,vel_ctrl,[.1,0,0]))
backward.bind('<ButtonPress-1>', lambda event: move(num_joints,robot_def,vel_ctrl,[-.1,0,0]))
up.bind('<ButtonPress-1>', lambda event: move(num_joints,robot_def,vel_ctrl,[0,0,.1]))
down.bind('<ButtonPress-1>', lambda event: move(num_joints,robot_def,vel_ctrl,[0,0,-.1]))
r_cl.bind('<ButtonPress-1>', lambda event: rotate(num_joints,0.1))
r_ct.bind('<ButtonPress-1>', lambda event: rotate(num_joints,-0.1))

left.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
right.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
forward.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
backward.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
up.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
down.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
r_cl.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
r_ct.bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))

left.pack()
right.pack()
forward.pack()
backward.pack()
up.pack()
down.pack()
gripper.pack()
roller.pack()
r_cl.pack()
r_ct.pack()

top.mainloop()
vel_ctrl.disable_velocity_mode()