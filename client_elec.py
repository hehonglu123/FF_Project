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
from temp_match import match

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


eef_angle=np.pi/2.
eef_orientation=R_ee.R_ee(np.pi/2.)
place_orientation=R_ee.R_ee(np.pi/2.)
fabric_position=np.array([-0.55,0.6,0.0155])
place_position=np.array([0.3,0.6,0.03])
place_offset=[0,0.02,0]	#offset wrt bottom fabric, [orientation angle, distance, placing orientation]

transformation=H42H3(H_ABB)


###fabric template
template_left=cv2.imread('client_yaml/piece0_left.png',cv2.IMREAD_UNCHANGED)
template_right=cv2.imread('client_yaml/piece0_right.png',cv2.IMREAD_UNCHANGED)

# url='rr+tcp://192.168.50.166:11111?service=m1k'
# m1k_obj = RRN.ConnectService(url)
# m1k_obj.setmode('A', 'SVMI')

def pick(p,orientation):
	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.5*np.ones(n), True)

	q=inv.inv(p,orientation)
	robot.jog_freespace(q, 0.5*np.ones(n), True)

	tool.close()
	time.sleep(2)
	# m1k_obj.setawgconstant('A',5)
	# m1k_obj.read(1000)
	# 
	# m1k_obj.setawgconstant('A',5)
	# m1k_obj.read(1000)

	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.1*np.ones(n), True)

def place(p,orientation):
	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	robot.jog_freespace(q, 0.5*np.ones(n), True)
	q=inv.inv(p,orientation)
	robot.jog_freespace(q, np.ones(n), True)


	tool.open()
	time.sleep(5)
	# m1k_obj.setawgconstant('A',0)
	# time.sleep(5)
	# m1k_obj.read(1000)

	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	robot.jog_freespace(q, np.ones(n), True)




pick(fabric_position,R_ee.R_ee(np.pi))	 

place(place_position,place_orientation)
