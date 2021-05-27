#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy, yaml
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('toolbox/')
from vel_emulate_sub import EmulatedVelocityControl
from general_robotics_toolbox import *    
from pixel2coord import pixel2coord
from fabric_detection import detection

inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)

with open('calibration/abb.yaml') as file:
	H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('calibration/camera_extrinsic.yaml') as file:
	realsense_param = yaml.load(file, Loader=yaml.FullLoader)
p_realsense=np.array(realsense_param['p'])
R_realsense=np.array(realsense_param['R'])

with open(r'client_yaml/client_'+robot_name+'.yaml') as file:
	robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
home=robot_yaml['home']
table_height=0.005
ROI=np.array([[160,600],[165,1153]])	#ROI [[r1,r2],[c1,c2]]
green=[30,51,1]
blue=[112,55,0]
white=[220,203,190]


def H42H3(H):
	H3=np.linalg.inv(H[:2,:2])
	H3=np.hstack((H3,-np.dot(H3,np.array([[H[0][-1]],[H[1][-1]]]))))
	H3=np.vstack((H3,np.array([0,0,1])))
	return H3

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
url='rr+tcp://localhost:25415?service=Multi_Cam_Service'


#Startup, connect, and pull out the camera from the objref    
Multi_Cam_obj=RRN.ConnectService(url)
image_consts = RRN.GetConstants('com.robotraconteur.image', Multi_Cam_obj)

#Connect the pipe FrameStream to get the PipeEndpoint p
rgb_cam=Multi_Cam_obj.get_cameras(0)
p=rgb_cam.frame_stream.Connect(-1)
#Set the callback for when a new pipe packet is received to the
#new_frame function
p.PacketReceivedEvent+=new_frame
try:
	rgb_cam.start_streaming()
except: 
	traceback.print_exc()
	pass


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

eef_angle=np.pi/2.
eef_orientation=R_ee.R_ee(np.pi/2.)
place_orientation=R_ee.R_ee(np.pi/2.)
fabric_position=np.array([-0.2,0.6,0.18])
place_position=np.array([0.3,0.5,0.02])
place_offset=[0,0.02,0]	#offset wrt bottom fabric, [orientation angle, distance, placing orientation]

transformation=H42H3(H_ABB)

def jog_joint(q):
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.1:
		qdot=1	*(q-vel_ctrl.joint_position())
		vel_ctrl.set_velocity_command(qdot)
	vel_ctrl.set_velocity_command(np.zeros((n,)))
	return
def jog_home():
	q=inv.inv(home,eef_orientation)
	jog_joint(q)
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



def move_till_switch(qd):
	#start performing fabric picking
	robot_state=state_w.TryGetInValue()
	if not robot_state[0]:
		sys.exit("robot not ready")

	
	q_cur=robot_state[1].joint_position
	while np.linalg.norm(q_cur-qd)>0.01:
		robot_state=state_w.TryGetInValue()
		q_cur=robot_state[1].joint_position
		tool_state=tool_state_w.TryGetInValue()	
		vel_ctrl.set_velocity_command(0.5*(qd-q_cur))
	vel_ctrl.set_velocity_command(np.zeros(n))
	return

def pick(p,orientation):
	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0,0.1]),orientation.tolist())
	jog_joint(q)
	qd=inv.inv(p,orientation)
	move_till_switch(qd)

	tool.close()
	time.sleep(0.5)
	#move up
	q=inv.inv(p+np.array([0,0,0.1]),orientation)
	jog_joint(q)

def shake(p):
	qd=inv.inv(p,orientation)
	qdot=qd-vel_ctrl.joint_position()
	vel_ctrl.set_velocity_command(10*qdot)
	time.sleep(0.15)
	vel_ctrl.set_velocity_command(-10*qdot)
	time.sleep(0.15)
	vel_ctrl.set_velocity_command(np.zeros(n))


def place(p,orientation):
	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	jog_joint(q)
	qd=inv.inv(p,orientation)
	move_till_switch(qd)
	tool.open()
	time.sleep(0.5)
	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	jog_joint(q)

def pick_fabric(color,frame,shift=False):
	roi_frame=frame[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
	(orientation,centroid)=detection(roi_frame,color)
	try:
		centroid[0]
	except: 
		return
	center=centroid[0]+ROI[:,0]

	#temp script
	if shift:
		offset_white=125		#100 in pixel
		center+=[offset_white*np.cos(orientation[0]),offset_white*np.sin(orientation[0])]
		orientation[0]+=np.pi/2
	#temp end
	if color==blue:
		orientation[0]+=np.pi
	p=pixel2coord(R_realsense,p_realsense,np.flip(center),0)
	p=np.dot(transformation,np.array([[p[0]],[p[1]],[1]]))
	p[2]=table_height
	pick(p.flatten(),R_ee.R_ee(orientation[0]))

def drag_place(p,orientation):
	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0.3,0.2]),orientation)
	jog_joint(q)
	q=inv.inv(p+np.array([0,0.3,0.05]),orientation)
	jog_joint(q)
	q=inv.inv(p+np.array([0,0,0.05]),orientation)
	jog_joint(q)
	qd=inv.inv(p,orientation)
	move_till_switch(qd)
	tool.open()
	time.sleep(0.5)
	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	jog_joint(q)



# def place_fabric_fusable(bottom_color,place_offset):
# 	roi_frame=frame[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
# 	(orientation,centroid)=detection(roi_frame,color)
# 	try:
# 		centroid[0]
# 	except: 
# 		return
# 	center=centroid[0]+ROI[:,0]
# 	p=pixel2coord(R_realsense,p_realsense,np.flip(center),0)
	

fabric_height=0.0001
tool.open()
while True:
	jog_home()
	#reset tool default state
	
	if (not current_frame is None):
		pick_fabric(white,current_frame,True)
		# place(place_position,place_orientation)
		drag_place(place_position,place_orientation)
		pick_fabric(green,current_frame)
		
		# place(place_position,place_orientation)


		roi_frame=current_frame[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
		(orientation,centroid)=detection(roi_frame,[220,203,190])
		try:
			centroid[0]
		except: 
			traceback.print_exc()
			sys.exit()
		center=centroid[0]+ROI[:,0]
		#temp script
		offset_green=[110,40]		#100 in pixel
		center+=[offset_green[0]*np.cos(orientation[0])-offset_green[1]*np.sin(orientation[0]),offset_green[0]*np.sin(orientation[0])+offset_green[1]*np.cos(orientation[0])]
		#temp end

		p=pixel2coord(R_realsense,p_realsense,np.flip(center),0)
		p=np.dot(transformation,np.array([[p[0]],[p[1]],[1]]))
		p[2]=table_height
		place(p.flatten(),R_ee.R_ee(orientation[0]-np.pi/45))



		pick_fabric(blue,current_frame)
		center=centroid[0]+ROI[:,0]
		#temp script
		offset_blue=[100,-75]		#100 in pixel
		center+=[offset_blue[0]*np.cos(orientation[0])-offset_blue[1]*np.sin(orientation[0]),offset_blue[0]*np.sin(orientation[0])+offset_blue[1]*np.cos(orientation[0])]
		#temp end

		p=pixel2coord(R_realsense,p_realsense,np.flip(center),0)
		p=np.dot(transformation,np.array([[p[0]],[p[1]],[1]]))
		p[2]=table_height
		place(p.flatten(),R_ee.R_ee(orientation[0]-np.pi/45))
		

