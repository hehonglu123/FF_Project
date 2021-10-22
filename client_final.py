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
from abb_def import *
from qpsolvers import solve_qp

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
	vel_ctrl.set_velocity_command(np.zeros((6,)))
	vel_ctrl.disable_velocity_mode() 


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


def pick(p,R,v):
	global robot, tool, m1k_obj
	print('go picking')
	q=inv(p+np.array([0,0,0.5]),R)
	jog_joint(q, 0.5)

	#move down 
	q=inv(p+np.array([0,0,0.2]),R)
	jog_joint(q, 0.2,[0,0,-0.01])
	# q=inv(p+np.array([0,0,0.1]),R)
	# jog_joint(q, 0.1)

	#pick
	q=inv(p,R)
	jog_joint(q,0.05)

	# tool.setf_param('voltage',RR.VarValue(v,'single'))
	m1k_obj.setawgconstant('A',v)
	time.sleep(3)
	

	#move up
	q=inv(p+np.array([0,0,0.1]),R)
	jog_joint(q, 0.1, [0,0,0.01])
	# q=inv(p+np.array([0,0,0.2]),R)
	# jog_joint(q, 0.1)
	q=inv(p+np.array([0,0,0.5]),R)
	jog_joint(q, 0.1)

def place(place_position,angle):
	global robot, tool, m1k_obj
	print('go placing')
	R=R_ee.R_ee(angle)
	#start joggging to initial pose
	q=inv(place_position+np.array([0,0,0.1]),R)
	jog_joint(q, 0.1)

	#move down 
	q=inv(place_position,R)
	jog_joint(q, 0.1)
	

	###turn off adhesion, turn on HV relay, pin down
	# tool.setf_param('voltage',RR.VarValue(0.,'single'))
	m1k_obj.setawgconstant('A',0.)
	time.sleep(0.1)
	tool.setf_param('relay',RR.VarValue(1,'int8'))

	tool.close()

	time.sleep(2.5)
	
	tool.open()

	time.sleep(0.5)
	

	#move up
	q=inv(place_position+np.array([0,0,0.1]),R)
	jog_joint(q, 0.05)

	#turn off HV relay
	tool.setf_param('relay',RR.VarValue(0,'int8'))

def	vision_check(ROI,ppu,template,vision_q):
	global cam, robot, halt_mode, jog_mode, position_mode, place_position_global
	print("vision check")

	jog_joint(vision_q,0.2)

	###clear buffer
	cam.capture_frame()


	current_frame=ImageToMat(cam.capture_frame())
	cv2.imwrite("vision_check.jpg",current_frame)
	roi_frame=cv2.cvtColor(current_frame[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)


	q=inv(place_position_global+np.array([0,0,0.1]),R_ee.R_ee(0))
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = jog_mode
	robot.jog_freespace(q,0.1*np.ones(6), False)

	angle,center=match_w_ori(roi_frame,template,0,'edge')


	
	# offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)/np.sqrt(ppu)
	offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)
	offset_p[0]=offset_p[0]/1.391666
	offset_p[1]=offset_p[1]/1.3825

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
		if np.max(np.abs(qdot))>0.6:
			qdot=np.zeros(6)
			print('too fast')
		vel_ctrl.set_velocity_command(qdot)

	except:
		traceback.print_exc()
	return

def	vision_check_fb(ROI,ppu,template,vision_q):
	global cam, robot, halt_mode, jog_mode, position_mode, place_position_global, state_w
	print("vision check")

	jog_joint(vision_q,0.5)

	###clear buffer
	cam.capture_frame()


	current_frame=ImageToMat(cam.capture_frame())
	cv2.imwrite("vision_check.jpg",current_frame)
	roi_frame=cv2.cvtColor(current_frame[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)

	angle,center=match_w_ori(roi_frame,template,0,'edge')
	offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)

	vel_ctrl.enable_velocity_mode()
	while np.linalg.norm(offset_p)>1:
		move(np.array([offset_p[1],-offset_p[0],0.])/1000.,np.eye(3))

		cam.capture_frame()
		current_frame=ImageToMat(cam.capture_frame())

		roi_frame=cv2.cvtColor(current_frame[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)

		angle,center=match_w_ori_single(roi_frame,template,np.radians(angle),'edge')
		offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)
		print(offset_p)

	vel_ctrl.disable_velocity_mode() 

	p_cur=fwd(state_w.InValue.joint_position).p
	p_vision=fwd(vision_q).p
	return p_vision-p_cur,np.radians(angle)



def place_slide(place_position,angle):
	global robot, tool, m1k_obj
	print('go placing')
	R=R_ee.R_ee(angle)
	#start joggging to initial pose
	q=inv(place_position+np.array([0,0,0.1]),R)
	jog_joint(q, 0.1)

	#move down 
	q=inv(place_position,R)
	jog_joint(q, 0.1)
	

	###turn off adhesion, pin down
	# tool.setf_param('voltage',RR.VarValue(0.,'single'))
	m1k_obj.setawgconstant('A',0.)
	tool.close()
	time.sleep(1.)
	tool.setf_param('relay',RR.VarValue(1,'int8'))
	###sliding
	q=inv(place_position+np.array([0.18,0,0.]),R)
	jog_joint(q, 0.1)

	tool.open()
	time.sleep(0.5)
	###move up
	q=inv(place_position+np.array([0.18,0,0.1]),R)
	jog_joint(q, 0.1)
	tool.setf_param('relay',RR.VarValue(0,'int8'))

def slide(place_position):
	global robot, tool
	q=inv(place_position+np.array([0,0,0.2]),R_ee.R_ee(0))
	jog_joint(q, 0.1)
	q=inv(place_position,R_ee.R_ee(0))
	jog_joint(q, 0.1)
	tool.close()
	###sliding
	q=inv(place_position+np.array([0.1,0,0.]),R_ee.R_ee(0))
	jog_joint(q, 0.1)
	tool.open()
	###move up
	q=inv(place_position+np.array([0.1,0,0.1]),R_ee.R_ee(0))
	jog_joint(q, 0.1)

def connect_failed(s, client_id, url, err):
    print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))


def main():
	global robot, tool, cam, vel_ctrl, halt_mode, jog_mode, position_mode, m1k_obj, place_position_global,state_w


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
	place_position_global=testbed_yaml['place_position']
	ROI=vision_yaml['ROI']
	ppu=vision_yaml['ppu']
	pins_height=np.array([0,0,0.015])

	try:	
		url='rr+tcp://pi_fuse:11111?service=m1k'
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

	#rpi relay
	try:
		tool_sub=RRN.SubscribeService('rr+tcp://pi_fuse:22222?service=tool')
		tool=tool_sub.GetDefaultClientWait(1)
		tool.open()
		tool.setf_param('relay',RR.VarValue(0,'int8'))
		# tool.setf_param('voltage',RR.VarValue(0.,'single'))
	except:
		traceback.print_exc()
		print('rpi not available')
		pass

	try:
		robot_sub=RRN.SubscribeService('rr+tcp://pi_fuse:58651?service=robot')
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
		pose=fwd(q)
		jog_joint(inv([pose.p[0],pose.p[1],0.6],pose.R), 0.3)



		##home
		jog_joint(inv(home,R_ee.R_ee(0)), 0.3)

		fabric_name='PD19_016C-FR-LFT-UP HICKEY V2 36'
		template=read_template('client_yaml/templates/'+fabric_name+'.jpg',fabric_dimension[fabric_name],ppu)
		stack_height1=np.array([0,0,0.003])
		pick(bin1_p+stack_height1,bin1_R,v=1.9)

		# offset_p,offset_angle=vision_check(ROI,ppu,template,vision_q)
		offset_p,offset_angle=vision_check_fb(ROI,ppu,template,vision_q)
		place(place_position_global-offset_p+pins_height,offset_angle)

		stack_height2=np.array([0,0,0.005])
		pick(bin2_p+stack_height2,bin2_R,v=1.6)
		# offset_p,offset_angle=vision_check(ROI,ppu,template,vision_q)
		offset_p,offset_angle=vision_check_fb(ROI,ppu,template,vision_q)
		place(place_position_global-offset_p+pins_height,offset_angle)
		##home
		jog_joint(inv(home,R_ee.R_ee(0)), 0.3)
		##reset chargepad
		tool.setf_param('relay',RR.VarValue(1,'int8'))
		time.sleep(1.)
		tool.setf_param('relay',RR.VarValue(0,'int8'))

	except:

		m1k_obj.EndSession()
		traceback.print_exc()
	m1k_obj.EndSession()

if __name__ == '__main__':
    main()
