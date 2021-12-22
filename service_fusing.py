#!/usr/bin/python3
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
import sys, os, time, argparse, traceback, cv2, copy, yaml, threading
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
		##################################IP configurations#######################################
		self.fusing_laptop='192.168.51.116'
		self.pi_fuse='192.168.51.25'
		self.robosewclient='192.168.51.61'
		self.my_laptop='192.168.51.181'

		##################################background threading#######################################
		self._streaming=False
		self._lock = threading.Lock()
		self._robot_ready=False
		self._robot_enabled=False
		self._robot_running=False
		self._robot_failure=False
		self._estop=False

		##################################RR param initialization###############################################
		self.current_ply_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
		self.current_interlining_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
		self.error_message_type=RRN.NewStructure("com.robotraconteur.eventlog.EventLogMessage")
		self.finish_signal_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FinishSignal")

		self.current_errors=[]

		self.current_ply_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR HICKEY 36'
		self.current_interlining_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR-INT HICKEY 36'
		

		##################################Load Local params######################################
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

		self.actuator_position={'bin1':None,'bin2':None,'robot':None}

		##################################Execution parameters#######################################		
		self.current_operation_count=0

		##################################Other RR services connection#######################################
		try:	
			url='rr+tcp://'+self.fusing_laptop+':11111?service=m1k'
			self.m1k_sub=RRN.SubscribeService(url)
			self.m1k_sub.ClientConnectFailed += self.connect_failed

			####get client object
			self.m1k_obj = self.m1k_sub.GetDefaultClientWait(1)
			self.m1k_obj.setvoltage(0)
		except:
			print('m1k not available')
			pass

		try:
			###camera connect
			url='rr+tcp://'+self.fusing_laptop+':59823?service=camera'
			#Startup, connect, and pull out the camera from the objref    
			self.cam_sub=RRN.SubscribeService(url)
			self.cam_sub.ClientConnectFailed += self.connect_failed
			####get client object
			self.cam = self.cam_sub.GetDefaultClientWait(1)
			cam_pipe=self.cam.frame_stream.Connect(-1)
			cam_pipe.PacketReceivedEvent+=self.new_frame
		except:
			print('camera not available')
			pass

		#tool
		try:
			###sensor_state [IDEC1,IDEC2,IDEC3,IDEC4,LOCK1,LOCK]
			self.tool_sub=RRN.SubscribeService('rr+tcp://'+self.fusing_laptop+':22222?service=tool')
			self.tool_state = self.tool_sub.SubscribeWire("tool_state")
			self.tool_sub.ClientConnectFailed += self.connect_failed
			self.tool=self.tool_sub.GetDefaultClientWait(1)
			self.tool.open()
			self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
			self.tool.setf_param('relay',RR.VarValue(0,'int8'))
			
		except:
			print('tool service not available')
			pass

		try:
			self.robot_sub=RRN.SubscribeService('rr+tcp://'+self.fusing_laptop+':58651?service=robot')
			self.robot_sub.ClientConnectFailed += self.connect_failed
			self.state_w = self.robot_sub.SubscribeWire("robot_state")
			self.cmd_w=self.robot_sub.SubscribeWire('position_command')

			self.robot=self.robot_sub.GetDefaultClientWait(1)
			##########Initialize robot constants
			robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", self.robot)
			self.halt_mode = robot_const["RobotCommandMode"]["halt"]
			self.position_mode = robot_const["RobotCommandMode"]["position_command"]
			self.state_flags_enum = robot_const['RobotStateFlags']


			self.vel_ctrl = EmulatedVelocityControl(self.robot,self.state_w, self.cmd_w)

			

			self.robot.command_mode = self.halt_mode
			time.sleep(0.1)
			self.robot.command_mode = self.position_mode

			#enable velocity mode
			self.vel_ctrl.enable_velocity_mode()

			self._robot_running=True

		except:
			self._robot_running=False
			print('robot not available')
			pass

	def initialize(self):

		try:
			#start getting sensor data
			self.StartStreaming()
			self._estop=False
			####m1k client object
			try:
				self.m1k_obj.setvoltage(0)
			except:
				self.m1k_obj = self.m1k_sub.GetDefaultClientWait(1)
				self.m1k_obj.setvoltage(0)

			####camera client object
			try:
				self.cam.start_streaming()
				self.cam.stop_streaming()
			except:
				self.cam = self.cam_sub.GetDefaultClientWait(1)
				cam_pipe=self.cam.frame_stream.Connect(-1)
				cam_pipe.PacketReceivedEvent+=self.new_frame
			
			###tool client object
			try:
				self.tool.setf_param('relay',RR.VarValue(1,'int8'))
				self.tool.close()
				time.sleep(1)
				self.tool.setf_param('relay',RR.VarValue(0,'int8'))
				self.tool.open()
			except:
				self.tool=self.tool_sub.GetDefaultClientWait(1)
				# self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
				self.tool.setf_param('relay',RR.VarValue(1,'int8'))
				self.tool.close()
				time.sleep(1)
				self.tool.setf_param('relay',RR.VarValue(0,'int8'))
				self.tool.open()

			###robot client object
			try:
				self.robot.command_mode = self.halt_mode
				time.sleep(0.1)
				self.robot.command_mode = self.position_mode
				#enable velocity mode
				self.vel_ctrl.enable_velocity_mode()
				self._robot_running=True
			except:
				print('Reconnect Robot')
				self.robot=self.robot_sub.GetDefaultClientWait(1)
				self.vel_ctrl = EmulatedVelocityControl(self.robot,self.state_w, self.cmd_w)

				##########Initialize robot constants
				robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", self.robot)
				self.halt_mode = robot_const["RobotCommandMode"]["halt"]
				self.position_mode = robot_const["RobotCommandMode"]["position_command"]

				self.robot.command_mode = self.halt_mode
				time.sleep(0.1)
				self.robot.command_mode = self.position_mode

				#enable velocity mode
				self.vel_ctrl.enable_velocity_mode()
				self._robot_running=True

			###temp, lift up
			q=self.state_w.InValue.joint_position
			p_cur=fwd(q).p
			self.jog_joint_movel([p_cur[0],p_cur[1],0.6], 0.2,threshold=0.05, acc_range=0.)

			##home
			self.jog_joint(inv(self.home,R_ee(0)), 0.5,threshold=0.1)
			self.vel_ctrl.set_velocity_command(np.zeros((6,)))

			return True
		except:
			self.trigger_error('initialization failed ',traceback.format_exc())
			return False


	###connection failed callback
	def connect_failed(self, s, client_id, url, err):
		error_msg="Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err)
		self.trigger_error('Connection Error',error_msg)

	#triggering pipe
	@property
	def trigger_fusing_system(self):
		return self._trigger_fusing_system
	@trigger_fusing_system.setter
	def trigger_fusing_system(self,value):
		self._trigger_fusing_system=value
		value.PipeConnectCallback=(self.p1_connect_callback)

	def p1_connect_callback(self,p):
		p.PacketReceivedEvent+=self.p1_packet_received

	def p1_packet_received(self,p):
		while p.Available:
			try:
				dat=p.ReceivePacket()
				self.current_operation_count=0
				self.execute(dat)
			except:
				self.trigger_error('execution failed ',traceback.format_exc())

	#sensor reading
	def threadfunc(self):
		while(self._streaming):
			time.sleep(0.2)
			with self._lock:
				# print(self._robot_ready,self._robot_enabled,self._robot_failure,self._robot_running)
				try:
					# print(self.tool_state.InValue.sensor[:6].astype(bool))
					self.sensor_readings.OutValue=self.tool_state.InValue.sensor[:6].astype(bool)
				except:
					pass
				###robot state checking
				try:
					self._robot_ready=False
					self._robot_enabled=False
					self._robot_failure=False
					for flag_name, flag_code in self.state_flags_enum.items():
						if flag_code & self.state_w.InValue.robot_state_flags != 0:
							print(flag_name)
							if flag_code==262144:
								self._robot_ready=True
							if flag_code==131072:
								self._robot_enabled=True
							if flag_code==2097152:
								self._robot_failure=True
					if self._robot_ready and self._robot_enabled and not self._robot_failure:
						self._robot_running=True
					else:
						self._robot_running=False

				except:
					self._robot_running=False

	def StartStreaming(self):
		if (self._streaming):
			return
		self._streaming=True
		t=threading.Thread(target=self.threadfunc)
		t.start()
	def StopStreaming(self):
		self._streaming=False

	###pressure actuator locks placeholders
	def actuate(self,position,action):

		return 
	###ESTOP
	def stop_fusing(self):
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))
		self.vel_ctrl.disable_velocity_mode()
		self._estop=True

		self.m1k_obj.setvoltage(0)
		self.tool.open()
		self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
		self.tool.setf_param('relay',RR.VarValue(1,'int8'))
		time.sleep(0.5)
		self.tool.setf_param('relay',RR.VarValue(0,'int8'))

		


	###error handling
	def trigger_error(self,title,error_msg):
		self.error_message_type.title=title
		self.error_message_type.message=error_msg
		self.finish_signal_type.current_errors=[self.error_message_type]
		self.finish_signal_type.finished=True

		print(error_msg)
		###send stop signal to WGC
		try:
			self.finish_signal.SendPacket(self.finish_signal_type)
		except:
			print('pipe not initialized yet')
			pass

		self.vel_ctrl.set_velocity_command(np.zeros((6,)))
		self.vel_ctrl.disable_velocity_mode()

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

			if self._robot_running==False:
				self.vel_ctrl.set_velocity_command(np.zeros((6,)))
				raise AssertionError('robot not ready')
				return

			if self._estop==True:
				self.vel_ctrl.set_velocity_command(np.zeros((6,)))
				raise AssertionError('Manual Estop')
				return
		


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

			if self._robot_running==False:
				self.vel_ctrl.set_velocity_command(np.zeros((6,)))
				raise AssertionError('robot not ready')
				return
			if self._estop==True:
				self.vel_ctrl.set_velocity_command(np.zeros((6,)))
				raise AssertionError('Manual Estop')
				return

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
		try:
			print('go picking')
			q=inv(p+np.array([0,0,0.3]),R)
			self.jog_joint(q, 1.2,threshold=0.002,dcc_range=0.4)

			
			#turn on voltage first
			# self.m1k_obj.setawgconstant('A',v)
			self.m1k_obj.setvoltage(v)
			# self.tool.setf_param('voltage',RR.VarValue(v,'single'))

			#move down 
			self.jog_joint_movel(p,0.3,threshold=0.002,acc_range=0.,dcc_range=0.1,Rd=R)

			self.vel_ctrl.set_velocity_command(np.zeros((6,)))

			
			
			# time.sleep(0.5)
			

			#move up
			self.jog_joint_movel(p+np.array([0,0,0.3]),0.45,acc_range=0.1,threshold=0.05)
		except:
			self.trigger_error("Robot not ready",traceback.format_exc())

	def pick_osc(self,p,R,v):
		print('go picking')
		q=inv(p+np.array([0,0,0.5]),R)
		self.jog_joint(q, 1.5,threshold=0.002,dcc_range=0.4)

		#move down 
		self.jog_joint_movel(p,0.3,threshold=0.002,acc_range=0.,dcc_range=0.1,Rd=R)

		self.vel_ctrl.set_velocity_command(np.zeros((6,)))

		#pick
		# self.m1k_obj.setawgconstant('A',v)
		self.m1k_obj.setvoltage(v)
		# self.tool.setf_param('voltage',RR.VarValue(v,'single'))
		time.sleep(0.5)

		###osc
		self.jog_joint_movel(p+np.array([0,0,0.023]),0.3,threshold=0.005,acc_range=0.,dcc_range=0.1,Rd=R)
		# self.m1k_obj.setawgconstant('A',0.)
		self.m1k_obj.setvoltage(0)
		# self.tool.setf_param('voltage',RR.VarValue(0,'single'))

		time.sleep(0.2)
		self.tool.setf_param('relay',RR.VarValue(1,'int8'))
		self.tool.close()
		time.sleep(2.)
		self.tool.open()
		self.tool.setf_param('relay',RR.VarValue(0,'int8'))
		self.jog_joint_movel(p,0.3,threshold=0.002,acc_range=0.,dcc_range=0.1,Rd=R)
		# self.m1k_obj.setawgconstant('A',v)
		self.m1k_obj.setvoltage(v)
		# self.tool.setf_param('voltage',RR.VarValue(v,'single'))
		

		#move up
		self.jog_joint_movel(p+np.array([0,0,0.3]),0.45,acc_range=0.1,threshold=0.05)



	def place(self,place_position,angle):
		print('go placing')
		R=np.dot(self.place_orientation,Rx(-angle))

		#start joggging to place position
		q=inv(place_position+self.pins_height,R)
		self.jog_joint(q, 0.7,threshold=0.001,dcc_range=0.1)
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))

		###turn off adhesion first, 
		# self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
		# self.m1k_obj.setawgconstant('A',0.)
		self.m1k_obj.setvoltage(0)
		###keep moving until perfectly in contact with metal plate
		#turn on HV relay, pin down
		q=inv(place_position,R)
		self.jog_joint(q, 0.1,threshold=0.001,dcc_range=0.05)
		self.tool.setf_param('relay',RR.VarValue(1,'int8'))

		self.tool.close()
		time.sleep(0.2)
		self.jog_joint_movel(place_position+self.pins_height, 0.2,threshold=0.002,dcc_range=0.1)
		# time.sleep(2)
		
		self.tool.open()

		time.sleep(0.1)
		

		#move up
		q=inv(place_position+np.array([0,0,0.1]),R)
		self.jog_joint(q, 0.6,threshold=0.15,dcc_range=0.12)

		#turn off HV relay
		self.tool.setf_param('relay',RR.VarValue(0,'int8'))


	def move(self,vd, ER):
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
		if np.max(np.abs(qdot))>0.8:
			qdot=np.zeros(6)
			print('too fast')
		self.vel_ctrl.set_velocity_command(qdot)

		return

	def	vision_check_fb(self,template,interlining=False):
		print("vision check")
		try:
			self.cam.start_streaming()
		except:
			traceback.print_exc()
			pass
		self.jog_joint(self.vision_q, 1.5,threshold=0.0001,dcc_range=0.4)

		#####brief stop for vision
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))
		self.vel_ctrl.disable_velocity_mode()
		time.sleep(0.5)

		###write for reference
		cv2.imwrite("vision_check.jpg",self.current_frame)
		roi_frame=cv2.cvtColor(self.current_frame[self.ROI[0]:self.ROI[1],self.ROI[2]:self.ROI[3]], cv2.COLOR_BGR2GRAY)

		angle,center=match_w_ori(roi_frame,template,0,'edge',interlining=interlining)
		###precise angle 
		angle,center=match_w_ori(roi_frame,template,np.radians(angle),'edge',angle_range=1.,angle_resolution=0.1,interlining=interlining)

		offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)
		self.vel_ctrl.enable_velocity_mode()

		###jog with large offset first

		while np.linalg.norm(offset_p)>3:
			###Robot Error
			if self._robot_running==False:
				self.vel_ctrl.set_velocity_command(np.zeros((6,)))
				raise AssertionError('Robot Error')
				return
			###Operator Stop
			if self._estop==True:
				self.vel_ctrl.set_velocity_command(np.zeros((6,)))
				raise AssertionError('Manual Estop')
				return
			try:
				roi_frame=cv2.cvtColor(self.current_frame[self.ROI[0]:self.ROI[1],self.ROI[2]:self.ROI[3]], cv2.COLOR_BGR2GRAY)
				angle,center=match_w_ori_single(roi_frame,template,np.radians(angle),'edge',interlining=interlining)
				offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)
				print(offset_p)
				self.move(np.array([offset_p[1],-offset_p[0],0.])/1000.,np.eye(3))
			except:
				self.trigger_error('Vision Error',traceback.format_exc())
				return

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
		# self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
		# self.m1k_obj.setawgconstant('A',0.)
		self.m1k_obj.setvoltage(0)

		###keep jogging down until perfect contact with metal plate
		q=inv(place_position,R)
		self.jog_joint(q, 0.1,threshold=0.001,dcc_range=0.05)

		self.tool.setf_param('relay',RR.VarValue(1,'int8'))
		self.tool.close()
		time.sleep(0.2)
		###move up more
		# self.jog_joint_movel(place_position+self.pins_height+np.array([0,0,0.035]), 0.01,threshold=0.001,acc_range=0.005,dcc_range=0.01)
		#move back down, pressing the plate
		self.jog_joint_movel(place_position+self.pins_height, 0.2, threshold=0.002,dcc_range=0.1)

		self.jog_joint_movel(place_position+self.pins_height/6, 0.1, threshold=0.002,dcc_range=0.1)


		###sliding

		now=time.time()
		while time.time()-now<7:
			self.move(np.array([0.08,0,0]),np.eye(3))

		self.tool.open()
		# vel_ctrl.set_velocity_command(np.zeros((6,)))
		time.sleep(0.5)
		###move up
		q=inv(place_position+np.array([0.3,0,0.1]),R)
		self.jog_joint(q, 0.4,threshold=0.1,dcc_range=0.12)
		self.tool.setf_param('relay',RR.VarValue(0,'int8'))


	def execute(self, stacks):

		for cur_stack in range(stacks):
		
			try:
				
				# self.fabric_template=read_template('client_yaml/templates/'+self.current_ply_fabric_type.fabric_name+'.jpg',self.fabric_dimension[self.current_ply_fabric_type.fabric_name],self.ppu)
				
				# self.stack_height1=np.array([0,0,0.00-cur_stack*0.00075])
				# self.pick(self.bin1_p+self.stack_height1,self.bin1_R,v=3.5)
				# # self.pick_osc(self.bin1_p+self.stack_height1,self.bin1_R,v=3.8)

				# offset_p,offset_angle=self.vision_check_fb(self.fabric_template)
				# ######no-vision block
				# # offset_p=np.array([0,0,0])
				# # offset_angle=0.
				# # ######no-vision block end
				# self.place(self.place_position-offset_p,offset_angle)


				self.interlining_template=read_template('client_yaml/templates/'+self.current_interlining_fabric_type.fabric_name+'.jpg',self.fabric_dimension[self.current_interlining_fabric_type.fabric_name],self.ppu)
				
				self.stack_height2=np.array([0,0,0.004-cur_stack*0.00045])
				self.pick(self.bin2_p+self.stack_height2,self.bin2_R,v=4.)
				# self.pick_osc(self.bin2_p+self.stack_height2,self.bin2_R,v=4.1)
				offset_p,offset_angle=self.vision_check_fb(self.interlining_template,interlining=True)
				######no-vision block
				# offset_p=np.array([0,0,0])
				# offset_angle=0.
				######no-vision block end
				self.place(self.place_position-offset_p,offset_angle)
				# self.place_slide(self.place_position-offset_p,offset_angle)

				##home
				self.jog_joint(inv(self.home,R_ee(0)), 0.5, threshold=0.1)
				##reset chargepad
				self.tool.setf_param('relay',RR.VarValue(1,'int8'))
				time.sleep(1.)
				self.tool.setf_param('relay',RR.VarValue(0,'int8'))

			except:
				self.vel_ctrl.disable_velocity_mode()
				# self.m1k_obj.EndSession()
				self.trigger_error('Execution Error',traceback.format_exc())
				return

			self.current_operation_count+=1

		self.vel_ctrl.set_velocity_command(np.zeros((6,)))


def main():
	with RR.ServerNodeSetup("fusing_service",12180) as node_setup:
		RRC.RegisterStdRobDefServiceTypes(RRN)
		RRN.RegisterServiceTypeFromFile("edu.rpi.robotics.fusing_system")

		try:
			fusing_pi_obj=fusing_pi()
			fusing_pi_obj.initialize()
		
			service_ctx = RRN.RegisterService("fusing_service","edu.rpi.robotics.fusing_system.FusingSystem",fusing_pi_obj)

			# fusing_pi_obj.execute(5)
			input('press enter to quit')

		except:
			traceback.print_exc()
			fusing_pi_obj.StopStreaming()
			fusing_pi_obj.vel_ctrl.disable_velocity_mode()
			# fusing_pi_obj.m1k_obj.EndSession()
		# print("Press ctrl+c to quit")

		fusing_pi_obj.StopStreaming()
		fusing_pi_obj.vel_ctrl.disable_velocity_mode()
		
		# fusing_pi_obj.m1k_obj.EndSession()

if __name__ == '__main__':
	main()