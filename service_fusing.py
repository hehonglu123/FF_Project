#!/usr/bin/python3
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
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


class fusing_pi(object):
	def __init__(self):
		self.current_ply_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
		self.current_interlining_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")

		self.current_frame=None

		###read yamls
		with open(r'client_yaml/testbed.yaml') as file:
			testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)
		with open(r'client_yaml/vision.yaml') as file:
			vision_yaml = yaml.load(file, Loader=yaml.FullLoader)
		with open('client_yaml/fabric.yaml') as file:
			self.fabric_dimension = yaml.load(file, Loader=yaml.FullLoader)

		self.home=testbed_yaml['home_p']
		self.bin1_p=testbed_yaml['bin1_p']
		self.bin1_R=np.array(testbed_yaml['bin1_R']).reshape((3,3))
		self.bin2_p=testbed_yaml['bin2_p']
		self.bin2_R=np.array(testbed_yaml['bin2_R']).reshape((3,3))
		self.vision_q=testbed_yaml['vision_q']
		self.place_position=testbed_yaml['place_p']
		self.place_orientation=np.array(testbed_yaml['place_R']).reshape((3,3))
		self.ROI=vision_yaml['ROI']
		self.ppu=vision_yaml['ppu']
		self.pins_height=np.array([0,0,0.02])

		try:	
			url='rr+tcp://192.168.51.25:11111?service=m1k'
			m1k_sub=RRN.SubscribeService(url)
			####get client object
			self.m1k_obj = m1k_sub.GetDefaultClientWait(1)
			self.m1k_obj.StartSession()
			self.m1k_obj.setmode('A', 'SVMI')
			self.m1k_obj.setawgconstant('A',0.)
		except:
			traceback.print_exc()
			print('m1k not available')
			pass


		###camera connect
		url='rr+tcp://robosewclient:59823?service=camera'
		#Startup, connect, and pull out the camera from the objref    
		self.cam=RRN.ConnectService(url)
		cam_pipe=self.cam.frame_stream.Connect(-1)
		cam_pipe.PacketReceivedEvent+=self.new_frame

		#rpi relay
		try:
			self.tool_sub=RRN.SubscribeService('rr+tcp://192.168.51.25:22222?service=tool')
			self.tool=self.tool_sub.GetDefaultClientWait(1)
			self.tool.open()
			self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
			self.tool.setf_param('relay',RR.VarValue(0,'int8'))
		except:
			traceback.print_exc()
			print('tool service not available')
			pass

		try:
			self.robot_sub=RRN.SubscribeService('rr+tcp://192.168.51.25:58651?service=robot')
			self.robot=self.robot_sub.GetDefaultClientWait(1)
			self.state_w = self.robot_sub.SubscribeWire("robot_state")
			cmd_w=self.robot_sub.SubscribeWire('position_command')
			self.vel_ctrl = EmulatedVelocityControl(self.robot,self.state_w, cmd_w)

			##########Initialize robot constants
			robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", self.robot)
			self.halt_mode = robot_const["RobotCommandMode"]["halt"]
			self.position_mode = robot_const["RobotCommandMode"]["position_command"]

			self.robot.command_mode = self.halt_mode
			time.sleep(0.1)
			self.robot.command_mode = self.position_mode

			#enable velocity mode
			self.vel_ctrl.enable_velocity_mode()
		except:
			print('robot not available')

	def jog_joint(self,q,max_v,threshold=0.01,dcc_range=0.1):

		gain=max(2*max_v,1)
		diff=q-self.vel_ctrl.joint_position()

		while np.linalg.norm(diff)>threshold:
			
			diff=q-self.vel_ctrl.joint_position()
			diff_norm=np.linalg.norm(diff)
			# qdot=np.where(np.abs(diff) > dcc_range, max_v*diff/np.linalg.norm(diff), gain*diff)
			if diff_norm<dcc_range:
				qdot=gain*diff
			else:
				qdot=max_v*diff/diff_norm
			self.vel_ctrl.set_velocity_command(qdot)
		


	def jog_joint_movel(self,p,max_v,threshold=0.001,acc_range=0.01,dcc_range=0.04,Rd=[]):

		gain=max(5*max_v,1)
		p_init=fwd(self.vel_ctrl.joint_position()).p
		diff=p-p_init
		time_temp=np.linalg.norm(diff)/max_v


		while np.linalg.norm(diff)>threshold:
			pose_cur=fwd(self.vel_ctrl.joint_position())
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
				self.move(v,np.eye(3))
			else:
				self.move(v,np.dot(pose_cur.R,Rd.T))


	def read_template(self,im_path,dimension,ppu):
		#load template
		template=cv2.imread(im_path, cv2.IMREAD_GRAYSCALE)
		
		template_ppu=len(template)*len(template[0])/(dimension[0]*dimension[1])

		scale=ppu/template_ppu

		template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

		#convert to binary
		template_binary= cv2.threshold(template, 40, 255, cv2.THRESH_BINARY)[1]

		return template_binary

	def new_frame(self,pipe_ep):

		#Loop to get the newest frame
		while (pipe_ep.Available > 0):
			#Receive the packet
			
			image=pipe_ep.ReceivePacket()
			#Convert the packet to an image and set the global variable
			self.current_frame=self.ImageToMat(image)

			return

	def ImageToMat(self,image):

		frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

		return frame2


	def pick(self,p,R,v):
		print('go picking')
		q=inv(p+np.array([0,0,0.5]),R)
		self.jog_joint(q, 1.5,threshold=0.002,dcc_range=0.4)

		#move down 
		self.jog_joint_movel(p,0.3,threshold=0.002,acc_range=0.,dcc_range=0.1,Rd=R)

		self.vel_ctrl.set_velocity_command(np.zeros((6,)))

		#pick
		self.m1k_obj.setawgconstant('A',v)
		self.tool.setf_param('voltage',RR.VarValue(v,'single'))
		time.sleep(0.5)
		

		#move up
		self.jog_joint_movel(p+np.array([0,0,0.3]),0.45,acc_range=0.2,threshold=0.05)

	def pick_osc(self,p,R,v):
		print('go picking')
		q=inv(p+np.array([0,0,0.5]),R)
		self.jog_joint(q, 1.5,threshold=0.002,dcc_range=0.4)

		#actuate pins
		self.jog_joint_movel(p+np.array([0,0,0.01]),0.3,threshold=0.005,acc_range=0.,dcc_range=0.1,Rd=R)
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))
		self.tool.close()
		time.sleep(1)
		self.tool.open()
		time.sleep(1)
		self.tool.close()
		time.sleep(1)
		self.tool.open()

		self.jog_joint_movel(p,0.3,threshold=0.002,acc_range=0.,dcc_range=0.1,Rd=R)
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))

		###oscilate
		self.m1k_obj.setawgconstant('A',v)
		self.tool.setf_param('voltage',RR.VarValue(v,'single'))
		self.jog_joint_movel(p+np.array([0,0,0.02]),0.3,threshold=0.005,acc_range=0.,dcc_range=0.1,Rd=R)
		self.jog_joint_movel(p,0.3,threshold=0.002,acc_range=0.,dcc_range=0.1,Rd=R)

		#pick
		self.m1k_obj.setawgconstant('A',v)
		self.tool.setf_param('voltage',RR.VarValue(v,'single'))
		time.sleep(0.5)
		

		#move up
		self.jog_joint_movel(p+np.array([0,0,0.3]),0.45,acc_range=0.2,threshold=0.05)



	def place(self,place_position,angle):
		print('go placing')
		R=np.dot(self.place_orientation,Rx(-angle))

		#start joggging to place position
		q=inv(place_position+self.pins_height,R)
		self.jog_joint(q, 0.7,threshold=0.001,dcc_range=0.1)
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))

		###turn off adhesion first, 
		self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
		self.m1k_obj.setawgconstant('A',0.)
		###keep moving until perfectly in contact with metal plate
		#turn on HV relay, pin down
		q=inv(place_position,R)
		self.jog_joint(q, 0.1,threshold=0.0005,dcc_range=0.1)
		self.tool.setf_param('relay',RR.VarValue(1,'int8'))

		self.tool.close()
		time.sleep(0.2)
		self.jog_joint_movel(place_position+self.pins_height, 0.01,threshold=0.001,dcc_range=0.1)
		# time.sleep(2)
		
		self.tool.open()

		time.sleep(0.1)
		

		#move up
		q=inv(place_position+np.array([0,0,0.1]),R)
		self.jog_joint(q, 0.6,threshold=0.15,dcc_range=0.12)

		#turn off HV relay
		self.tool.setf_param('relay',RR.VarValue(0,'int8'))


	def move(self,vd, ER):
		try:
			w=1.
			Kq=.01*np.eye(6)    #small value to make sure positive definite
			KR=np.eye(3)        #gains for position and orientation error

			q_cur=self.vel_ctrl.joint_position()
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
			self.vel_ctrl.set_velocity_command(qdot)

		except:
			traceback.print_exc()
		return

	def	vision_check_fb(self):
		print("vision check")
		try:
			self.cam.start_streaming()
		except:
			traceback.print_exc()
			pass
		self.jog_joint(self.vision_q, 1.5,threshold=0.0001,dcc_range=0.4)

		# return np.array([0,0,0]),0

		#####brief stop for vision
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))
		self.vel_ctrl.disable_velocity_mode()
		time.sleep(0.5)

		###write for reference
		cv2.imwrite("vision_check.jpg",self.current_frame)
		roi_frame=cv2.cvtColor(self.current_frame[self.ROI[0]:self.ROI[1],self.ROI[2]:self.ROI[3]], cv2.COLOR_BGR2GRAY)

		angle,center=match_w_ori(roi_frame,self.template,0,'edge')
		###precise angle 
		angle,center=match_w_ori(roi_frame,self.template,np.radians(angle),'edge',angle_range=1.,angle_resolution=0.1)

		offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)
		self.vel_ctrl.enable_velocity_mode()

		###jog with large offset first

		while np.linalg.norm(offset_p)>4:


			roi_frame=cv2.cvtColor(self.current_frame[self.ROI[0]:self.ROI[1],self.ROI[2]:self.ROI[3]], cv2.COLOR_BGR2GRAY)
			angle,center=match_w_ori_single(roi_frame,self.template,np.radians(angle),'edge')
			offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)
			print(offset_p)
			# print(offset_p,np.linalg.norm(offset_p))
			if np.linalg.norm(offset_p)>10:
				self.move(np.array([offset_p[1],-offset_p[0],0.])/1500.,np.eye(3))
			else:
				self.move(np.array([offset_p[1],-offset_p[0],0.])/5000.,np.eye(3))

		self.vel_ctrl.set_velocity_command(np.zeros((6,)))

		p_cur=fwd(self.state_w.InValue.joint_position).p
		p_vision=fwd(self.vision_q).p

		self.cam.stop_streaming()

		print('offset_angle: ',angle)
		return p_vision-p_cur,np.radians(angle)



	def place_slide(self,place_position,angle):
		print('go placing')
		R=np.dot(self.place_orientation,Rx(-angle))
		#start joggging to place position
		q=inv(place_position+self.pins_height,R)
		self.jog_joint(q, 0.7,threshold=0.001,dcc_range=0.1)
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))

		###turn off adhesion first
		self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
		self.m1k_obj.setawgconstant('A',0.)

		###keep jogging down until perfect contact with metal plate
		q=inv(place_position,R)
		self.jog_joint(q, 0.1,threshold=0.0005,dcc_range=0.1)

		self.tool.setf_param('relay',RR.VarValue(1,'int8'))
		self.tool.close()
		time.sleep(0.2)
		#move up more
		self.jog_joint_movel(place_position+self.pins_height+np.array([0,0,0.035]), 0.01,threshold=0.001,acc_range=0.005,dcc_range=0.01)
		#move back down, pressing the plate
		self.jog_joint_movel(place_position+self.pins_height, 0.01,threshold=0.001,dcc_range=0.1)
		###sliding

		now=time.time()
		while time.time()-now<6:
			self.move(np.array([0.08,0,0]),np.eye(3))

		self.tool.open()
		# vel_ctrl.set_velocity_command(np.zeros((6,)))
		time.sleep(0.5)
		###move up
		q=inv(place_position+np.array([0.3,0,0.1]),R)
		self.jog_joint(q, 0.4,threshold=0.1,dcc_range=0.12)
		self.tool.setf_param('relay',RR.VarValue(0,'int8'))


	def initialize(self):
		###temp, lift up
		q=self.state_w.InValue.joint_position
		p_cur=fwd(q).p
		self.jog_joint_movel([p_cur[0],p_cur[1],0.6], 0.2,threshold=0.05, acc_range=0.)

		##home
		self.jog_joint(inv(self.home,R_ee(0)), 0.5,threshold=0.1)

	def execute(self):

		
		try:
			self.fabric_name='PD19_016C-FR-LFT-LWR HICKEY V2 44'
			self.template=read_template('client_yaml/templates/'+self.fabric_name+'.jpg',self.fabric_dimension[self.fabric_name],self.ppu)
			
			self.stack_height1=np.array([0,0,-0.00])
			self.pick(self.bin1_p+self.stack_height1,self.bin1_R,v=4.5)
			offset_p,offset_angle=self.vision_check_fb()
			self.place(self.place_position-offset_p,offset_angle)


			self.stack_height2=np.array([0,0,0.004])
			self.pick(self.bin2_p+self.stack_height2,self.bin2_R,v=4.5)
			offset_p,offset_angle=self.vision_check_fb()
			# self.place(self.place_position-offset_p,offset_angle)
			self.place_slide(self.place_position-offset_p,offset_angle)

			##home
			self.jog_joint(inv(self.home,R_ee(0)), 0.5, threshold=0.1)
			##reset chargepad
			self.tool.setf_param('relay',RR.VarValue(1,'int8'))
			time.sleep(1.)
			self.tool.setf_param('relay',RR.VarValue(0,'int8'))

		except:
			self.vel_ctrl.disable_velocity_mode()
			self.m1k_obj.EndSession()
			self.traceback.print_exc()


def main():
	with RR.ServerNodeSetup("fusing_service",12180) as node_setup:
		RRC.RegisterStdRobDefServiceTypes(RRN)
		RRN.RegisterServiceTypeFromFile("edu.rpi.robotics.fusing_system")

		fusing_pi_obj=fusing_pi()
		fusing_pi_obj.initialize()

		for i in range(5):
			
			fusing_pi_obj.execute()

	
		service_ctx = RRN.RegisterService("fusing_service","edu.rpi.robotics.fusing_system.FusingSystem",fusing_pi_obj)

		# print("Press ctrl+c to quit")

		fusing_pi_obj.vel_ctrl.disable_velocity_mode()
		fusing_pi_obj.m1k_obj.EndSession()

if __name__ == '__main__':
	main()
