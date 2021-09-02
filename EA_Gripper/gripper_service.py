from relay_lib_seeed import *
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
import numpy as np 
import time, traceback, threading, signal, board, busio, adafruit_vl53l0x
from gpiozero import Button
from pysmu import Session, Mode

class EA_Gripper(object):
	def __init__(self):

		self.tool_state_type=RRN.NewStructure("com.robotraconteur.robotics.tool.ToolState")
		self._date_time_util = DateTimeUtil(RRN)

		# i2c = busio.I2C(board.SCL, board.SDA)
		# self.sensor_28 = adafruit_vl53l0x.VL53L0X(i2c,address=0x28)
		# self.sensor_29 = adafruit_vl53l0x.VL53L0X(i2c,address=0x29)

		self._streaming=False
		self._lock = threading.Lock()
		self.tool_state_type=RRN.NewStructure("com.robotraconteur.robotics.tool.ToolState")
		self._date_time_util = DateTimeUtil(RRN)

		###M1k setting
		self.session = Session(ignore_dataflow=True, queue_size=100000)
		# queue size can be changed, set to 1 Sec of data at 100KSPS
		self.num_dev = self.session.scan() # get number of connected boards
		if self.num_dev == 0:
			print("No M1ks Found")

		else:
			# get first (only?) device
			self.device = self.session.devices[0]
			#initialize default mode to HI_Z
			self.CHA = self.device.channels['A']
			self.CHA.mode = Mode.SVMI
			self.session.start(0)
			self.CHA.constant(0)

	def open(self):
		relay_on(1)
		relay_off(2)

	def close(self):
		relay_off(1)
		relay_on(2)

	def setf_param(self,param_name, value):
		if param_name=='voltage':
			print('setting voltage to: ', value.data[0])
			self.CHA.constant(value.data[0])

	#sensor reading
	def threadfunc(self):
		while(self._streaming):
			time.sleep(0.2)
			with self._lock:
				try:
					ToolState=self.tool_state_type
					self.sensor_list=[]
					# self.sensor_list=[self.sensor_28.range,self.sensor_29.range]
					# print(self.sensor_list)
					ToolState.sensor=self.sensor_list
					ToolState.ts=self._date_time_util.TimeSpec3Now()
					self.tool_state.OutValue=ToolState
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





with RR.ServerNodeSetup("electroadhesion_gripper",22222) as node_setup:
	RRC.RegisterStdRobDefServiceTypes(RRN)

	gripper_inst=EA_Gripper()
	gripper_inst.StartStreaming()
	service_ctx = RRN.RegisterService("tool","com.robotraconteur.robotics.tool.Tool",gripper_inst)

	print("Press ctrl+c to quit")
	signal.sigwait([signal.SIGTERM,signal.SIGINT])
	gripper_inst.StopStreaming()
	gripper_inst.session.end()
