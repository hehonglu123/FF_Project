#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy, yaml
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('../toolbox/')
from vel_emulate_sub import EmulatedVelocityControl
from general_robotics_toolbox import *    
from pixel2coord import *
from fabric_detection import detection
from temp_match import match

inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)

with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
	robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
home=robot_yaml['home']
table_height=0.005
ROI=np.array([[160,600],[165,1153]])	#ROI [[r1,r2],[c1,c2]]

ROI1=np.array([[160,650],[100,550]])	#ROI [[r1,r2],[c1,c2]]
ROI2=np.array([[160,650],[850,1300]])	#ROI [[r1,r2],[c1,c2]]


def read_template(im_path):
	###fabric template
	template=cv2.imread(im_path,cv2.IMREAD_UNCHANGED)
	mask=np.where(template[:,:,-1]>0, 1, 0)
	#calc avg template color
	non_zeros=np.count_nonzero(template[:,:,-1])
	B=np.sum(template[:,:,0]*mask[:,:])/non_zeros
	G=np.sum(template[:,:,1]*mask[:,:])/non_zeros
	R=np.sum(template[:,:,2]*mask[:,:])/non_zeros
	avg_color=[B,G,R]
	return avg_color,template


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

#rpi relay
try:
	tool_sub=RRN.SubscribeService('rr+tcp://192.168.50.115:22222?service=tool')
	tool=tool_sub.GetDefaultClientWait(1)
except:
	print('rpi relay not available')
	pass


robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
state_w = robot_sub.SubscribeWire("robot_state")


##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
state_flags_enum = robot_const['RobotStateFlags']
halt_mode = robot_const["RobotCommandMode"]["halt"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode

#parameter setup
n= len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
robot_def=Robot(H,np.transpose(P),np.zeros(n))


eef_angle=0.
eef_orientation=R_ee.R_ee(0)
place_orientation=R_ee.R_ee(0)
pick_position=np.array([-0.5, 0.654, 0.135])
discharge_position=np.array([0.55, 0.58, 0.143])

place_position=np.array([0.0556,0.613,0.14])



url='rr+tcp://192.168.50.166:11111?service=m1k'
m1k_obj = RRN.ConnectService(url)
m1k_obj.StartSession()

m1k_obj.setmode('A', 'SVMI')
m1k_obj.setawgconstant('A',0.)


def discharge(p,orientation):
	
	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.3*np.ones(n), True)


	#pick
	q=inv.inv(p,orientation)
	robot.jog_freespace(q, 0.1*np.ones(n), True)

	time.sleep(5)

	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.1*np.ones(n), True)

def pick(p,orientation):


	


	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.3*np.ones(n), True)

	# tool.setf_param('elec',RR.VarValue(True,'bool'))
	m1k_obj.setawgconstant('A',5.)

	#pick
	q=inv.inv(p,orientation)
	robot.jog_freespace(q, 0.1*np.ones(n), True)

	
	time.sleep(4)
	


	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.1*np.ones(n), True)

def place(p,orientation):

	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	robot.jog_freespace(q, 0.2*np.ones(n), True)

	
	#move down 
	q=inv.inv(p+np.array([0,0,0.019]),orientation)
	robot.jog_freespace(q, 0.1*np.ones(n), True)

	###turn off adhesion, pin down
	m1k_obj.setawgconstant('A',0.)
	# tool.setf_param('elec',RR.VarValue(False,'bool'))
	tool.close()
	time.sleep(2)
	###pin up
	tool.open()
	

	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	robot.jog_freespace(q, 0.2*np.ones(n), True)

	

##pin up, adhesion off first
# tool.setf_param('elec',RR.VarValue(False,'bool'))
m1k_obj.setawgconstant('A',0.)
tool.open()
##home
robot.jog_freespace(inv.inv(home,eef_orientation), 0.3*np.ones(n), True)


try:
	while True:
		try:

			pick(place_position,R_ee.R_ee(eef_angle))
			
			place(place_position,R_ee.R_ee(eef_angle))
			# time.sleep(5)
			m1k_obj.setawgconstant('A',0.)

			# tool.setf_param('elec',RR.VarValue(False,'bool'))

			discharge(discharge_position,R_ee.R_ee(eef_angle))

		except:
			m1k_obj.EndSession()
			traceback.print_exc()
		
except:
	m1k_obj.EndSession()
	traceback.print_exc()



m1k_obj.EndSession()
