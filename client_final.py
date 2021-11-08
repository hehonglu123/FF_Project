#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy, yaml
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('toolbox/')
from vel_emulate_sub import EmulatedVelocityControl
from general_robotics_toolbox import *    
from vision import *
from abb_def import *
from qpsolvers import solve_qp
from R_abb import *

def jog_joint(q,max_v,threshold=0.01,dcc_range=0.1):
	global vel_ctrl

	gain=max(2*max_v,1)
	diff=q-vel_ctrl.joint_position()

	while np.linalg.norm(diff)>threshold:
		
		diff=q-vel_ctrl.joint_position()
		diff_norm=np.linalg.norm(diff)
		# qdot=np.where(np.abs(diff) > dcc_range, max_v*diff/np.linalg.norm(diff), gain*diff)
		if diff_norm<dcc_range:
			qdot=gain*diff
		else:
			qdot=max_v*diff/diff_norm
		vel_ctrl.set_velocity_command(qdot)
	


def jog_joint_movel(p,max_v,threshold=0.001,acc_range=0.01,dcc_range=0.04,Rd=[]):
	global vel_ctrl

	gain=max(5*max_v,1)
	p_init=fwd(vel_ctrl.joint_position()).p
	diff=p-p_init
	time_temp=np.linalg.norm(diff)/max_v


	while np.linalg.norm(diff)>threshold:
		pose_cur=fwd(vel_ctrl.joint_position())
		diff=p-pose_cur.p
		diff2=np.linalg.norm(pose_cur.p-p_init)

		diff_norm=np.linalg.norm(diff)
		diff2_norm=np.linalg.norm(diff2)

		v_temp=max_v*diff/diff_norm

		if diff_norm<dcc_range:
			v=gain*diff
		elif diff2_norm<acc_range:
			v=diff2_norm*v_temp/acc_range
		else:
			v=v_temp
		###correcting orientation
		if len(Rd)==0:
			move(v,np.eye(3))
		else:
			move(v,np.dot(pose_cur.R,Rd.T))


def read_template(im_path,dimension,ppu):
	#load template
	template=cv2.imread(im_path, cv2.IMREAD_GRAYSCALE)
	
	template_ppu=len(template)*len(template[0])/(dimension[0]*dimension[1])

	scale=ppu/template_ppu

	template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

	#convert to binary
	template_binary= cv2.threshold(template, 40, 255, cv2.THRESH_BINARY)[1]

	return template_binary

def new_frame(pipe_ep):
	global current_frame, now

	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame=ImageToMat(image)

		return

def ImageToMat(image):

	frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

	return frame2


def pick(p,R,v):
	global robot, tool, m1k_obj
	print('go picking')
	q=inv(p+np.array([0,0,0.5]),R)
	jog_joint(q, 1.5,threshold=0.002,dcc_range=0.4)

	#move down 
	jog_joint_movel(p,0.3,threshold=0.002,acc_range=0.,dcc_range=0.1,Rd=R)

	vel_ctrl.set_velocity_command(np.zeros((6,)))

	#pick
	# m1k_obj.setawgconstant('A',v)
	tool.setf_param('voltage',RR.VarValue(v,'single'))
	time.sleep(0.5)
	

	#move up
	jog_joint_movel(p+np.array([0,0,0.3]),0.45,acc_range=0.2,threshold=0.05)



def place(place_position,angle):
	global robot, tool, m1k_obj, place_orientation_global, pins_height
	print('go placing')
	R=np.dot(place_orientation_global,Rx(-angle))

	#start joggging to place position
	q=inv(place_position+pins_height,R)
	jog_joint(q, 0.7,threshold=0.001,dcc_range=0.1)
	vel_ctrl.set_velocity_command(np.zeros((6,)))

	###turn off adhesion first, 
	tool.setf_param('voltage',RR.VarValue(0.,'single'))
	# m1k_obj.setawgconstant('A',0.)
	###keep moving until perfectly in contact with metal plate
	#turn on HV relay, pin down
	q=inv(place_position,R)
	jog_joint(q, 0.1,threshold=0.0005,dcc_range=0.1)
	tool.setf_param('relay',RR.VarValue(1,'int8'))

	tool.close()
	time.sleep(0.2)
	jog_joint_movel(place_position+pins_height, 0.01,threshold=0.001,dcc_range=0.1)
	# time.sleep(2)
	
	tool.open()

	time.sleep(0.1)
	

	#move up
	q=inv(place_position+np.array([0,0,0.1]),R)
	jog_joint(q, 0.3,threshold=0.1,dcc_range=0.12)

	#turn off HV relay
	tool.setf_param('relay',RR.VarValue(0,'int8'))


def move(vd, ER):
	global vel_ctrl, robot_def
	try:
		w=1.
		Kq=.01*np.eye(6)    #small value to make sure positive definite
		KR=np.eye(3)        #gains for position and orientation error

		q_cur=vel_ctrl.joint_position()
		J=jacobian(q_cur)       #calculate current Jacobian
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
		qdot=solve_qp(H, f)
		###For safty, make sure robot not moving too fast
		if np.max(np.abs(qdot))>1.:
			qdot=np.zeros(6)
			print('too fast')
		vel_ctrl.set_velocity_command(qdot)

	except:
		traceback.print_exc()
	return

def	vision_check_fb(ROI,ppu,template,vision_q):
	global cam, state_w, current_frame
	print("vision check")
	try:
		cam.start_streaming()
	except:
		traceback.print_exc()
		pass
	jog_joint(vision_q, 1.5,threshold=0.0001,dcc_range=0.4)

	###brief stop for vision
	vel_ctrl.set_velocity_command(np.zeros((6,)))
	vel_ctrl.disable_velocity_mode()
	time.sleep(0.5)

	###write for reference
	cv2.imwrite("vision_check.jpg",current_frame)
	roi_frame=cv2.cvtColor(current_frame[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)

	angle,center=match_w_ori(roi_frame,template,0,'edge')
	###precise angle 
	angle,center=match_w_ori(roi_frame,template,np.radians(angle),'edge',angle_range=1.,angle_resolution=0.1)

	offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)
	vel_ctrl.enable_velocity_mode()

	###jog with large offset first

	while np.linalg.norm(offset_p)>2:


		roi_frame=cv2.cvtColor(current_frame[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)
		angle,center=match_w_ori_single(roi_frame,template,np.radians(angle),'edge')
		offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)

		# print(offset_p,np.linalg.norm(offset_p))
		if np.linalg.norm(offset_p)>10:
			move(np.array([offset_p[1],-offset_p[0],0.])/1500.,np.eye(3))
		else:
			move(np.array([offset_p[1],-offset_p[0],0.])/5000.,np.eye(3))

	vel_ctrl.set_velocity_command(np.zeros((6,)))

	p_cur=fwd(state_w.InValue.joint_position).p
	p_vision=fwd(vision_q).p

	cam.stop_streaming()

	print('offset_angle: ',angle)
	return p_vision-p_cur,np.radians(angle)



def place_slide(place_position,angle):
	global robot, tool, m1k_obj, place_orientation_global, pins_height
	print('go placing')
	R=np.dot(place_orientation_global,Rx(-angle))
	#start joggging to place position
	q=inv(place_position+pins_height,R)
	jog_joint(q, 0.7,threshold=0.001,dcc_range=0.1)
	vel_ctrl.set_velocity_command(np.zeros((6,)))

	###turn off adhesion first
	tool.setf_param('voltage',RR.VarValue(0.,'single'))
	# m1k_obj.setawgconstant('A',0.)

	###keep jogging down until perfect contact with metal plate
	q=inv(place_position,R)
	jog_joint(q, 0.1,threshold=0.0005,dcc_range=0.1)

	tool.setf_param('relay',RR.VarValue(1,'int8'))
	tool.close()
	time.sleep(0.2)
	#move up more
	jog_joint_movel(place_position+pins_height+np.array([0,0,0.03]), 0.01,threshold=0.001,dcc_range=0.1)
	#move back down, pressing the plate
	jog_joint_movel(place_position+pins_height, 0.01,threshold=0.001,dcc_range=0.1)
	###sliding

	now=time.time()
	while time.time()-now<6:
		move(np.array([0.08,0,0]),np.eye(3))

	tool.open()
	# vel_ctrl.set_velocity_command(np.zeros((6,)))
	time.sleep(0.5)
	###move up
	q=inv(place_position+np.array([0.3,0,0.1]),R)
	jog_joint(q, 0.3,threshold=0.1,dcc_range=0.12)
	tool.setf_param('relay',RR.VarValue(0,'int8'))


def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))


def main():
	global robot, tool, cam, vel_ctrl, halt_mode, jog_mode, position_mode, m1k_obj, place_position_global,state_w, current_frame, place_orientation_global, pins_height

	current_frame=None

	###read yamls
	with open(r'client_yaml/testbed.yaml') as file:
		testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)
	with open(r'client_yaml/vision.yaml') as file:
		vision_yaml = yaml.load(file, Loader=yaml.FullLoader)
	with open('client_yaml/fabric.yaml') as file:
		fabric_dimension = yaml.load(file, Loader=yaml.FullLoader)

	home=testbed_yaml['home_p']
	bin1_p=testbed_yaml['bin1_p']
	bin1_R=np.array(testbed_yaml['bin1_R']).reshape((3,3))
	bin2_p=testbed_yaml['bin2_p']
	bin2_R=np.array(testbed_yaml['bin2_R']).reshape((3,3))
	vision_q=testbed_yaml['vision_q']
	place_position_global=testbed_yaml['place_p']
	place_orientation_global=np.array(testbed_yaml['place_R']).reshape((3,3))
	ROI=vision_yaml['ROI']
	ppu=vision_yaml['ppu']
	pins_height=np.array([0,0,0.02])

	try:	
		url='rr+tcp://192.168.50.253:11111?service=m1k'
		m1k_sub=RRN.SubscribeService(url)
		####get client object
		m1k_obj = m1k_sub.GetDefaultClientWait(1)
		m1k_obj.StartSession()
		m1k_obj.setmode('A', 'SVMI')
		m1k_obj.setawgconstant('A',0.)
	except:
		print('m1k not available')
		pass
	# m1k_sub.ClientConnectFailed += connect_failed


	###camera connect
	url='rr+tcp://robosewclient:59823?service=camera'
	#Startup, connect, and pull out the camera from the objref    
	cam=RRN.ConnectService(url)
	cam_pipe=cam.frame_stream.Connect(-1)
	cam_pipe.PacketReceivedEvent+=new_frame

	#rpi relay
	try:
		tool_sub=RRN.SubscribeService('rr+tcp://192.168.50.253:22222?service=tool')
		tool=tool_sub.GetDefaultClientWait(1)
		tool.open()
		tool.setf_param('relay',RR.VarValue(0,'int8'))
		tool.setf_param('voltage',RR.VarValue(0.,'single'))
	except:
		traceback.print_exc()
		print('rpi not available')
		pass

	try:
		robot_sub=RRN.SubscribeService('rr+tcp://192.168.50.253:58651?service=robot')
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

		#enable velocity mode
		vel_ctrl.enable_velocity_mode()

		###temp, lift up
		q=state_w.InValue.joint_position
		p_cur=fwd(q).p
		jog_joint_movel([p_cur[0],p_cur[1],0.6], 0.2,threshold=0.05, acc_range=0.)



		##home
		jog_joint(inv(home,R_ee(0)), 0.5,threshold=0.1)

		fabric_name='PD19_016C-FR-LFT-UP HICKEY V2 36'
		template=read_template('client_yaml/templates/'+fabric_name+'.jpg',fabric_dimension[fabric_name],ppu)
		
		stack_height1=np.array([0,0,0.003])
		pick(bin1_p+stack_height1,bin1_R,v=5.)
		offset_p,offset_angle=vision_check_fb(ROI,ppu,template,vision_q)
		place(place_position_global-offset_p,offset_angle)

		# stack_height1=np.array([0,0,0.003])
		# pick(bin1_p+stack_height1,bin1_R,v=5.)
		# offset_p,offset_angle=vision_check_fb(ROI,ppu,template,vision_q)
		# place(place_position_global-offset_p,offset_angle)

		stack_height2=np.array([0,0,0.005])
		pick(bin2_p+stack_height2,bin2_R,v=4.5)
		offset_p,offset_angle=vision_check_fb(ROI,ppu,template,vision_q)
		# place(place_position_global-offset_p,offset_angle)
		place_slide(place_position_global-offset_p,offset_angle)
		##home
		jog_joint(inv(home,R_ee(0)), 0.3)
		##reset chargepad
		tool.setf_param('relay',RR.VarValue(1,'int8'))
		time.sleep(1.)
		tool.setf_param('relay',RR.VarValue(0,'int8'))

	except:
		vel_ctrl.disable_velocity_mode()
		m1k_obj.EndSession()
		traceback.print_exc()
	m1k_obj.EndSession()
	vel_ctrl.disable_velocity_mode()

if __name__ == '__main__':
	main()
