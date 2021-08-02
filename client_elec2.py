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
from pixel2coord import *
from fabric_detection import detection
from temp_match import *

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
with open(r'client_yaml/vision.yaml') as file:
	vision_yaml = yaml.load(file, Loader=yaml.FullLoader)
ROI=np.array(vision_yaml['ROI']).reshape((2,2))		#[[r1,r2],[c1,c2]]


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

    frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

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
###oak1 connect
url='rr+tcp://localhost:59823?service=camera'
#Startup, connect, and pull out the camera from the objref    
c=RRN.ConnectService(url)

#Connect the pipe FrameStream to get the PipeEndpoint pipe
pipe=c.frame_stream.Connect(-1)

#Set the callback for when a new pipe packet is received to the
#new_frame function
pipe.PacketReceivedEvent+=new_frame
try:
    c.start_streaming()
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
pick_position1=np.array([-0.53749414, 0.69276434, 0.13])
pick_position2=np.array([0.614, 0.54770172, 0.13778046])

place_position=np.array([0.04843907,0.59446204,0.14366202])
place_offset=[0,0.02,0]	#offset wrt bottom fabric, [orientation angle, distance, placing orientation]

transformation=H42H3(H_ABB)



url='rr+tcp://192.168.50.166:11111?service=m1k'
m1k_obj = RRN.ConnectService(url)
m1k_obj.StartSession()

m1k_obj.setmode('A', 'SVMI')
m1k_obj.setawgconstant('A',0.)

#convert cognex frame to robot frame
def conversion(x,y,height):
	p=np.dot(transformation,np.array([[x],[y],[1]])).flatten()
	p[2]=height
	return p

def discharge():
	robot.jog_freespace(inv.inv(home,eef_orientation), 0.5*np.ones(n), True)
	# robot.jog_freespace([2.2097185,   1.00693821, -0.54655,     1.96079976,  0.9655184,   1.00660412], 0.5*np.ones(n), True)
	time.sleep(20)
	# robot.jog_freespace(inv.inv(home,eef_orientation), 0.5*np.ones(n), True)

def check_offset(template):
	###go to vision place
	robot.jog_freespace(inv.inv(home+np.array([0,0,0.05]),eef_orientation), 0.1*np.ones(n), True)

	q=[ 1.49419622,  0.78283241, -0.14415147, -0.1775408,  -0.64601098,  1.69674787]
	robot.jog_freespace(q, 0.1*np.ones(n), True)
	### fabric detection
	global current_frame
	if (not current_frame is None):
		roi_frame=current_frame[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]

	
		angle,center_temp=match_w_ori(roi_frame,template,[0],'edge')

		center=np.flip(center+np.flip(ROI[:,0]))

		#draw dots	
		cv2.circle(current_frame, tuple(np.flip(center).astype(int)), 10,(0,0,255), -1)		
		current_frame = cv2.putText(current_frame, str(p[0])+','+str(p[1])+' ,'+str(angle), org = tuple(np.flip(center).astype(int)), 
	           fontScale = 1, fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL,color = (255, 0, 0), thickness = 2, lineType=cv2.LINE_AA)


		current_frame = cv2.rectangle(current_frame, (ROI[1][0],ROI[0][0]), (ROI[1][1],ROI[0][1]), color = (255, 0, 0), thickness=2)


		cv2.imshow("Image",current_frame)
		cv2.waitKey(0)


def pick(p,orientation,v=5.):


	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.1*np.ones(n), True)

	# ##neutralize
	# # m1k_obj.setawgconstant('A',0)
	# q=inv.inv(p,orientation)
	# robot.jog_freespace(q, 0.1*np.ones(n), True)

	# time.sleep(5)
	# #move up
	# q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	# robot.jog_freespace(q, 0.1*np.ones(n), True)


	#pick
	q=inv.inv(p,orientation)
	robot.jog_freespace(q, 0.1*np.ones(n), True)

	# tool.setf_param('elec',RR.VarValue(True,'bool'))
	m1k_obj.setawgconstant('A',v)
	time.sleep(3)

	# tool.setf_param('elec',RR.VarValue(False,'bool'))
	


	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation.tolist())
	robot.jog_freespace(q, 0.1*np.ones(n), True)

def place(p,orientation):

	#start joggging to initial pose
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	robot.jog_freespace(q, 0.2*np.ones(n), True)

	# m1k_obj.setawgconstant('A',0.)
	#move down 
	q=inv.inv(p+np.array([0,0,0.016]),orientation)
	robot.jog_freespace(q, 0.1*np.ones(n), True)

	###turn off adhesion, pin down
	# tool.setf_param('elec',RR.VarValue(False,'bool'))
	tool.close()
	time.sleep(5)
	###pin up
	tool.open()
	

	#move up
	q=inv.inv(p+np.array([0,0,0.2]),orientation)
	robot.jog_freespace(q, 0.2*np.ones(n), True)

	


def pp_fabric(temp_path,pick_position,v=5.):
	

	###alwasy picking at fixed position
	pick(pick_position,R_ee.R_ee(eef_angle),v)

	angle,offset=check_offset(read_template(temp_path))

	# ###fix difference in placing
	# # print(place_position+(pick_position-p_fabric))
	# place(place_position+offset,R_ee.R_ee(eef_angle-np.radians(angle)))
	# discharge()


##pin up, adhesion off first
# tool.setf_param('elec',RR.VarValue(False,'bool'))
m1k_obj.setawgconstant('A',0.)
# tool.open()


# ##home
robot.jog_freespace(inv.inv(home,eef_orientation), 0.1*np.ones(n), True)


try:
	pp_fabric('client_yaml/template0.png',pick_position1,v=5.)

	# pp_fabric('client_yaml/template1.png',pick_position1,ROI1,v=5.)



	
except:
	m1k_obj.EndSession()
	pipe.Close()
	c.stop_streaming()
	traceback.print_exc()



m1k_obj.EndSession()
pipe.Close()
c.stop_streaming()
