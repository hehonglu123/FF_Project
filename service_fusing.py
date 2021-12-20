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
		self.fusing_laptop='192.168.51.188'
		self.pi_fuse='192.168.51.25'
		self.robosewclient='192.168.51.61'
		self.my_laptop='192.168.51.181'

		##################################sensor background threading#######################################
		self._streaming=False
		self._lock = threading.Lock()


		##################################RR param initialization###############################################
		self.current_ply_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
		self.current_interlining_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
		self.error_message_type=RRN.NewStructure("com.robotraconteur.eventlog.EventLogMessage")
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


		##################################Other RR services connection#######################################
		try:	
			url='rr+tcp://'+self.fusing_laptop+':11111?service=m1k'
			m1k_sub=RRN.SubscribeService(url)
			####get client object
			self.m1k_obj = m1k_sub.GetDefaultClientWait(1)
			self.m1k_obj.setvoltage(0)
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
			###sensor_state [IDEC1,IDEC2,IDEC3,IDEC4,LOCK1,LOCK]
			self.tool_sub=RRN.SubscribeService('rr+tcp://'+self.pi_fuse+':22222?service=tool')
			self.tool=self.tool_sub.GetDefaultClientWait(1)
			self.tool.open()
			self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
			self.tool.setf_param('relay',RR.VarValue(0,'int8'))
			self.sensor_state = self.tool_sub.SubscribeWire("tool_state")
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

			#start getting sensor data
			self.StartStreaming()
		except:
			print('robot not available')


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
				self.execute(dat.number_of_operations)
			except:
				self.trigger_error(traceback.format_exc())

	#sensor reading
	def threadfunc(self):
		while(self._streaming):
			time.sleep(0.2)
			with self._lock:
				try:
					self.sensor_readings.OutValue=self.sensor_state[:6]
				except:
					traceback.print_exc()

	def StartStreaming(self):
		if (self._streaming):
			raise RR.InvalidOperationException("Already streaming")
		self._streaming=True
		t=threading.Thread(target=self.threadfunc)
		t.start()
	def StopStreaming(self):
		if (not self._streaming):
			raise RR.InvalidOperationException("Not streaming")
		self._streaming=False


	###ESTOP
	def stop_fusing(self):
		self.vel_ctrl.set_velocity_command(np.zeros((6,)))
		# self.m1k_obj.setawgconstant('A',0.)
		self.m1k_obj.setvoltage(0)
		self.tool.open()
		self.tool.setf_param('voltage',RR.VarValue(0.,'single'))
		self.tool.setf_param('relay',RR.VarValue(0,'int8'))

		


	###error handling
	def trigger_error(self,error_msg):
		self.error_message_type.message=error_msg
		self.current_errors=[self.error_message_type.message]
		self.trigger_fusing_system.SendPacket(self.current_errors)
		print(error_msg)

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

		frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(i