#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy, yaml
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('toolbox/')
from vel_emulate_sub import EmulatedVelocityControl
from general_robotics_toolbox import *    
from temp_match import *

inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)

def jog_joint(q,max_v,vd=[]):
	global vel_ctrl
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()

	diff=q-vel_ctrl.joint_position()
	time_temp=np.linalg.norm(diff)/max_v

	# if vd:
	# 	J=inv.jacobian(q)

	# 	while np.linalg.norm(q-vel_ctrl.joint_position())>0.01:
	# 		qdot_temp=np.dot(np.linalg.inv(J),np.vstack((np.zeros((3,1)),np.array(vd).reshape((3,1))))).flatten()

	# 		diff=q-vel_ctrl.joint_position()
	# 		qdot=np.where(np.abs(diff) > 0.05, qdot_temp, diff)
	# 		print(qdot)
	# 		vel_ctrl.set_velocity_command(qdot)

	# else:	
	qdot_temp=np.clip((1.15*diff)/time_temp,-max_v,max_v)
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.01:
		
		diff=q-vel_ctrl.joint_position()
		qdot=np.where(np.abs(diff) > 0.05, qdot_temp, diff)

		vel_ctrl.set_velocity_command(qdot)


def read_template(im_path,dimension,ppu):
	#load template
	template=cv2.imread(im_path, cv2.IMREAD_GRAYSCALE)
	
	template_ppu=len(template)*len(template[0])/(dimension[0]*dimension[1])

	scale=ppu/template_ppu

	template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

	#convert to binary
	template_binary= cv2.threshold(template, 40, 255, cv2.THRESH_BINARY)[1]

	return template_binary
 
def ImageToMat(image):

    frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

    return frame2
current_frame=None

def new_frame(pipe_ep):
    global current_frame, now

    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        
        image=pipe_ep.ReceivePacket()
        #Convert the packet to an image and set the global variable
        current_frame=cv2.rotate(ImageToMat(image), cv2.ROTATE_180)

        return


def pick(p,R,v):
	global robot, tool
	print('go picking')
	q=inv.inv(p+np.array([0,0,0.5]),R)
	jog_joint(q, 0.3)

	#move down 
	q=inv.inv(p+np.array([0,0,0.2]),R)
	jog_joint(q, 0.1,[0,0,-0.01])
	# q=inv.inv(p+np.array([0,0,0.1]),R)
	# jog_joint(q, 0.1)

	#pick
	q=inv.inv(p,R)
	jog_joint(q,0.05)

	tool.setf_param('voltage',RR.VarValue(v,'single'))
	time.sleep(3)
	

	#move up
	q=inv.inv(p+np.array([0,0,0.1]),R)
	jog_joint(q, 0.1, [0,0,0.01])
	# q=inv.inv(p+np.array([0,0,0.2]),R)
	# jog_joint(q, 0.1)
	q=inv.inv(p+np.array([0,0,0.5]),R)
	jog_joint(q, 0.1)

def place(place_position,angle):
	global robot, tool
	print('go placing')
	R=R_ee.R_ee(angle)
	#start joggging to initial pose
	q=inv.inv(place_position+np.array([0,0,0.1]),R)
	jog_joint(q, 0.1)

	#move down 
	q=inv.inv(place_position,R)
	jog_joint(q, 0.1)
	

	###turn off adhesion, pin down
	tool.setf_param('voltage',RR.VarValue(0.,'single'))

	tool.close()
	time.sleep(2)
	tool.open()
	

	#move up
	q=inv.inv(place_position+np.array([0,0,0.1]),R)
	jog_joint(q, 0.1)

def	vision_check(ROI,ppu,template,vision_p,vision_q):
	global current_frame, robot

	q=inv.inv(vision_p+np.array([0.15,0,0.15]),R_ee.R_ee(0))
	jog_joint(q, 0.3)

	jog_joint(vision_q,0.2)

	cv2.imwrite("vision_check.jpg",current_frame)
	roi_frame=cv2.cvtColor(current_frame[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)
	image_copy=copy.deepcopy(current_frame)

	q=inv.inv(vision_p+np.array([0.15,0,0.15]),R_ee.R_ee(0))
	# jog_joint(q, 0.3)
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = jog_mode
	robot.jog_freespace(q,0.3*np.ones(6), wait=False)

	angle,center=match_w_ori(roi_frame,template,0.,'edge')


	
	offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)/ppu

	print(offset_p,angle)
	# ###show final results
	cv2.circle(roi_frame, (int(center[0]),int(center[1])),10,(0,0,255), -1)			
	cv2.imshow("image", roi_frame)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = position_mode

	return np.array([offset_p[1],-offset_p[0],0.])/1000.,np.radians(angle)

def slide(place_position):
	global robot, tool
	q=inv.inv(place_position+np.array([0,0,0.2]),R_ee.R_ee(0))
	jog_joint(q, 0.1)
	q=inv.inv(place_position,R_ee.R_ee(0))
	jog_joint(q, 0.1)
	tool.close()
	###sliding
	q=inv.inv(place_position+np.array([0.1,0,0.]),R_ee.R_ee(0))
	jog_joint(q, 0.1)
	tool.open()
	###move up
	q=inv.inv(place_position+np.array([0.1,0,0.1]),R_ee.R_ee(0))
	jog_joint(q, 0.1)

def main():
	global robot, tool, vel_ctrl


	###read yamls
	with open(r'client_yaml/testbed.yaml') as file:
		testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)
	with open(r'client_yaml/vision.yaml') as file:
		vision_yaml = yaml.load(file, Loader=yaml.FullLoader)
	with open('client_yaml/fabric.yaml') as file:
		fabric_dimension = yaml.load(file, Loader=yaml.FullLoader)

	home=testbed_yaml['home']
	bin1_p=testbed_yaml['bin1_p']
	bin1_R=np.array(testbed_yaml['bin1_R']).reshape((3,3))
	bin2_p=testbed_yaml['bin2_p']
	bin2_R=np.array(testbed_yaml['bin2_R']).reshape((3,3))
	vision_q=testbed_yaml['vision_q']
	vision_p=testbed_yaml['vision_p']
	place_position=testbed_yaml['place_position']
	ROI=vision_yaml['ROI']
	ppu=vision_yaml['ppu']




	###realsense connect
	url='rr+tcp://localhost:59823?service=camera'
	#Startup, connect, and pull out the camera from the objref    
	c=RRN.ConnectService(url)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	p=c.frame_stream.Connect(-1)

	#Set the callback for when a new pipe packet is received to the
	#new_frame function
	p.PacketReceivedEvent+=new_frame
	try:
	    c.start_streaming()
	except: 
	    traceback.print_exc()
	    pass

	#rpi relay
	try:
		tool_sub=RRN.SubscribeService('rr+tcp://localhost:22222?service=tool')
		tool=tool_sub.GetDefaultClientWait(1)
		tool.open()
	except:
		print('rpi relay not available')
		pass

	robot_sub=RRN.SubscribeService('rr+tcp://localhost:58651?service=robot')
	robot=robot_sub.GetDefaultClientWait(1)
	state_w = robot_sub.SubscribeWire("robot_state")
	cmd_w=robot_sub.SubscribeWire('position_command')
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)

	##########Initialize robot constants
	robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
	halt_mode = robot_const["RobotCommandMode"]["halt"]
	position_mode = robot_const["RobotCommandMode"]["position_command"]
	jog_mode = robot_const["RobotCommandMode"]["jog"]

	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = position_mode

	###temp, lift up
	q=state_w.InValue.joint_position
	pose=inv.fwd(q)
	jog_joint(inv.inv([pose.p[0],pose.p[1],0.6],pose.R), 0.3)



	##home
	jog_joint(inv.inv(home,R_ee.R_ee(0)), 0.3)


	template=read_template('client_yaml/FR-LF-UP.jpg',fabric_dimension['FR-LF-UP'],ppu)
	pick(bin1_p,bin1_R,v=0)
	# offset_p,offset_angle=vision_check(ROI,ppu,template,vision_p,vision_q)
	# place(place_position+offset_p,offset_angle)

	# template=read_template('client_yaml/FR-LF-UP.jpg',fabric_dimension['FR-LF-UP'],ppu)
	# pick(bin2_p,bin2_R,v=2.5)
	# offset_p,offset_angle=vision_check(ROI,ppu,template,vision_p,vision_q)
	# place(place_position+offset_p,offset_angle)

	# slide(place_position)
		



if __name__ == '__main__':
    main()
