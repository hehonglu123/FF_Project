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
from general_robotics_toolbox import *    



def Rx(theta):
	return np.array(([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]]))
def Ry(theta):
	return np.array(([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]]))
def Rz(theta):
	return np.array(([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]))

global vel_ctrl, robot_def


#Accept the names of the robots from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str)
args, _ = parser.parse_known_args()
robot_name=args.robot_name

#rpi relay
try:
	tool_sub=RRN.SubscribeService('rr+tcp://pi_fuse:22222?service=tool')
	tool=tool_sub.GetDefaultClientWait(1)
except:
	print('rpi gripper service not available')
	pass
try:	
	url='rr+tcp://pi_fuse:11111?service=m1k'
	m1k_obj = RRN.ConnectService(url)
	m1k_obj.StartSession()
	m1k_obj.setmode('A', 'SVMI')
	m1k_obj.setawgconstant('A',0.)
except:
	print('m1k not available')
	pass

#connect robot services
url='rr+tcp://pi_fuse:58651?service=robot'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
state_w = robot_sub.SubscribeWire("robot_state")
#get params of robots
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


top=Tk()
top.title("ABB1200 elec QP jogger")
jobid = None
def gripper_ctrl():


	if gripper.config('relief')[-1] == 'sunken':
		tool.setf_param('voltage',RR.VarValue(0.,'single'))
		# m1k_obj.setawgconstant('A',0.)
		gripper.config(relief="raised")
		gripper.configure(bg='red')
		gripper.configure(text='gripper off')

	else:
		tool.setf_param('voltage',RR.VarValue(3.5,'single'))
		# m1k_obj.setawgconstant('A',4.)
		gripper.config(relief="sunken")
		gripper.configure(bg='green')
		gripper.configure(text='gripper on')
	return

def pin_ctrl():

	if pin.config('relief')[-1] == 'sunken':
		tool.open()
		pin.config(relief="raised")
		pin.configure(bg='red')
		pin.configure(text='pin up')

	else:
		tool.close()
		pin.config(relief="sunken")
		pin.configure(bg='green')
		pin.configure(text='pin down')
	return

def relay_ctrl():

	if relay.config('relief')[-1] == 'sunken':
		tool.setf_param('relay',RR.VarValue(0,'int8'))
		relay.config(relief="raised")
		relay.configure(bg='red')
		relay.configure(text='relay off')

	else:
		tool.setf_param('relay',RR.VarValue(1,'int8'))
		relay.config(relief="sunken")
		relay.configure(bg='green')
		relay.configure(text='relay on')
	return


def movej(qdot):
	global jobid

	if np.max(np.abs(speed.get()*qdot))>1.0:
		qdot=np.zeros(6)
		print('too fast')

	vel_ctrl.set_velocity_command(speed.get()*qdot)
	jobid = top.after(10, lambda: movej(qdot))
	return


def move(vd, ER):
	global jobid, vel_ctrl, robot_def
	try:
		w=1.
		Kq=.01*np.eye(6)    #small value to make sure positive definite
		KR=np.eye(3)        #gains for position and orientation error

		q_cur=vel_ctrl.joint_position()
		J=robotjacobian(robot_def,q_cur)       #calculate current Jacobian
		Jp=J[3:,:]
		JR=J[:3,:] 
		H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)

		H=(H+np.transpose(H))/2


		k,theta = R2rot(ER)
		k=np.array(k)
		s=np.sin(theta/2)*k         #eR2
		wd=-np.dot(KR,s)  
		f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)
		###Don't put bound here, will affect cartesian motion outcome
		qdot=speed.get()*solve_qp(H, f)
		###For safty, make sure robot not moving too fast
		if np.max(np.abs(qdot))>1.0:
			qdot=np.zeros(6)
			print('too fast')
		vel_ctrl.set_velocity_command(qdot)

		jobid = top.after(10, lambda: move(vd, ER))
	except:
		traceback.print_exc()
	return

def stop():
	global jobid,vel_ctrl
	top.after_cancel(jobid)
	vel_ctrl.set_velocity_command(np.zeros((6,)))
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

	label.config(text = flags_text + "\n\n" + joint_text)

	label.after(250, update_label)

top.title = "Robot State"

###speed control
speed= Scale(orient='vertical', label='speed control',from_=1., length=500,resolution=0.1, to=10.)
speed.pack(side=RIGHT)


label = Label(top, fg = "black", justify=LEFT)
label.pack()
label.after(250,update_label)




left=Button(top,text='left')
right=Button(top,text='right')
forward=Button(top,text='forward')
backward=Button(top,text='backward')
up=Button(top,text='up')
down=Button(top,text='down')

Rx_n=Button(top,text='Rx_n')
Rx_p=Button(top,text='Rx_p')
Ry_n=Button(top,text='Ry_n')
Ry_p=Button(top,text='Ry_p')
Rz_n=Button(top,text='Rz_n')
Rz_p=Button(top,text='Rz_p')

j1_n=Button(top,text='j1_n')
j1_p=Button(top,text='j1_p')
j2_n=Button(top,text='j2_n')
j2_p=Button(top,text='j2_p')
j3_n=Button(top,text='j3_n')
j3_p=Button(top,text='j3_p')
j4_n=Button(top,text='j4_n')
j4_p=Button(top,text='j4_p')
j5_n=Button(top,text='j5_n')
j5_p=Button(top,text='j5_p')
j6_n=Button(top,text='j6_n')
j6_p=Button(top,text='j6_p')



gripper=Button(top,text='gripper off',command=lambda: gripper_ctrl(),bg='red')
pin=Button(top,text='pin up',command=lambda: pin_ctrl(),bg='red')
relay=Button(top,text='relay off',command=lambda: relay_ctrl(),bg='red')


left.bind('<ButtonPress-1>', lambda event: move([0,.02,0],np.eye(3)))
right.bind('<ButtonPress-1>', lambda event: move([0,-.02,0],np.eye(3)))
forward.bind('<ButtonPress-1>', lambda event: move([.02,0,0],np.eye(3)))
backward.bind('<ButtonPress-1>', lambda event: move([-.02,0,0],np.eye(3)))
up.bind('<ButtonPress-1>', lambda event: move([0,0,.02],np.eye(3)))
down.bind('<ButtonPress-1>', lambda event: move([0,0,-.02],np.eye(3)))

Rx_n.bind('<ButtonPress-1>', lambda event: move([0.,0.,0.],Rx(+0.1)))
Rx_p.bind('<ButtonPress-1>', lambda event: move([0.,0.,0.],Rx(-0.1)))
Ry_n.bind('<ButtonPress-1>', lambda event: move([0.,0.,0.],Ry(+0.1)))
Ry_p.bind('<ButtonPress-1>', lambda event: move([0.,0.,0.],Ry(-0.1)))
Rz_n.bind('<ButtonPress-1>', lambda event: move([0.,0.,0.],Rz(+0.1)))
Rz_p.bind('<ButtonPress-1>', lambda event: move([0.,0.,0.],Rz(-0.1)))

j1_n.bind('<ButtonPress-1>', lambda event: movej(np.array([-0.1,0.,0.,0.,0.,0.])))
j1_p.bind('<ButtonPress-1>', lambda event: movej(np.array([+0.1,0.,0.,0.,0.,0.])))
j2_n.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,-0.1,0.,0.,0.,0.])))
j2_p.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,+0.1,0.,0.,0.,0.])))
j3_n.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,-0.1,0.,0.,0.])))
j3_p.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,+0.1,0.,0.,0.])))
j4_n.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,0.,-0.1,0.,0.])))
j4_p.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,0.,+0.1,0.,0.])))
j5_n.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,0.,0.,-0.1,0.])))
j5_p.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,0.,0.,+0.1,0.])))
j6_n.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,0.,0.,0.,-0.1])))
j6_p.bind('<ButtonPress-1>', lambda event: movej(np.array([0.,0.,0.,0.,0.,+0.1])))


left.bind('<ButtonRelease-1>', lambda event: stop())
right.bind('<ButtonRelease-1>', lambda event: stop())
forward.bind('<ButtonRelease-1>', lambda event: stop())
backward.bind('<ButtonRelease-1>', lambda event: stop())
up.bind('<ButtonRelease-1>', lambda event: stop())
down.bind('<ButtonRelease-1>', lambda event: stop())

Rx_n.bind('<ButtonRelease-1>', lambda event: stop())
Rx_p.bind('<ButtonRelease-1>', lambda event: stop())
Ry_n.bind('<ButtonRelease-1>', lambda event: stop())
Ry_p.bind('<ButtonRelease-1>', lambda event: stop())
Rz_n.bind('<ButtonRelease-1>', lambda event: stop())
Rz_p.bind('<ButtonRelease-1>', lambda event: stop())

j1_n.bind('<ButtonRelease-1>', lambda event: stop())
j1_p.bind('<ButtonRelease-1>', lambda event: stop())
j2_n.bind('<ButtonRelease-1>', lambda event: stop())
j2_p.bind('<ButtonRelease-1>', lambda event: stop())
j3_n.bind('<ButtonRelease-1>', lambda event: stop())
j3_p.bind('<ButtonRelease-1>', lambda event: stop())
j4_n.bind('<ButtonRelease-1>', lambda event: stop())
j4_p.bind('<ButtonRelease-1>', lambda event: stop())
j5_n.bind('<ButtonRelease-1>', lambda event: stop())
j5_p.bind('<ButtonRelease-1>', lambda event: stop())
j6_n.bind('<ButtonRelease-1>', lambda event: stop())
j6_p.bind('<ButtonRelease-1>', lambda event: stop())



gripper.pack()
pin.pack()
relay.pack()

left.pack(in_=top, side=LEFT)
right.pack(in_=top, side=RIGHT)
forward.pack(in_=top, side=LEFT)
backward.pack(in_=top, side=RIGHT)
up.pack(in_=top, side=LEFT)
down.pack(in_=top, side=RIGHT)

Rx_n.pack()
Rx_p.pack()
Ry_n.pack()
Ry_p.pack()
Rz_n.pack()
Rz_p.pack()

j1_n.pack(in_=top, side=LEFT)
j1_p.pack(in_=top, side=LEFT)
j2_n.pack(in_=top, side=LEFT)
j2_p.pack(in_=top, side=LEFT)
j3_n.pack(in_=top, side=LEFT)
j3_p.pack(in_=top, side=LEFT)
j4_n.pack(in_=top, side=LEFT)
j4_p.pack(in_=top, side=LEFT)
j5_n.pack(in_=top, side=LEFT)
j5_p.pack(in_=top, side=LEFT)
j6_n.pack(in_=top, side=LEFT)
j6_p.pack(in_=top, side=LEFT)

try:
	top.mainloop()
except:
	vel_ctrl.disable_velocity_mode()
	m1k_obj.EndSession()


vel_ctrl.disable_velocity_mode()
m1k_obj.EndSession()