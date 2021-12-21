import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
import numpy as np 
import time, traceback, threading, signal#, board, busio, adafruit_vl53l0x, adafruit_sht31d
from io_test import RAPID
# import RPi.GPIO as GPIO

class EA_Gripper(object):
	def __init__(self):

		self.tool_state_type=RRN.NewStructure("com.robotraconteur.robotics.tool.ToolState")
		self._date_time_util = DateTimeUtil(RRN)

		###distance sensor
		# i2c = busio.I2C(board.SCL, board.SDA)
		# self.sensor_28 = adafruit_vl53l0x.VL53L0X(i2c,address=0x28)
		# self.sensor_29 = adafruit_vl53l0x.VL53L0X(i2c,address=0x29)
		###ht sensor
		# i2c2 = board.I2C()
		# self.ht_sensor = adafruit_sht31d.SHT31D(i2c2)

		self._streaming=False
		self._lock = threading.Lock()
		self.tool_state_type=RRN.NewStructure("com.robotraconteur.robotics.tool.ToolState")
		self._date_time_util = DateTimeUtil(RRN)

		###RPI PWM for HV control
		# GPIO.setwarnings(False)			#disable warnings
		# GPIO.setmode(GPIO.BCM)		#set pin numbering system
		# GPIO.setup(20,GPIO.OUT)
		# self.pi_pwm = GPIO.PWM(20,1000)		#create PWM instance with frequency
		# self.pi_pwm.start(0)

		###ABB DIO
		self.rapid = RAPID(base_url='http://192.168.51.26:80')



	def open(self):
		self.rapid.set_digital_io('valve1', 1) 
		self.rapid.set_digital_io('valve2', 0)

	def close(self):
		self.rapid.set_digital_io('valve1', 0) 
		self.rapid.set_digital_io('valve2', 1) 

	def setf_param(self,param_name, value):
		if param_name=='voltage':
			print('setting voltage to: ', value.data[0])
			# self.pi_pwm.ChangeDutyCycle(value.data[0]*100/5)
		if param_name=='relay':
			print('change relay to: ', value.data[0])
			self.rapid.set_digital_io('relay', value.data[0]) 

	#sensor reading
	def threadfunc(self):
		while(self._streaming):
			time.sleep(0.2)
			with self._lock:
				try:
					ToolState=self.tool_state_type
					IDEC1=self.rapid.get_digital_io('IDEC1')
					IDEC2=self.rapid.get_digital_io('IDEC2')
					IDEC3=self.rapid.get_digital_io('IDEC3')
					IDEC4=self.rapid.get_digital_io('IDEC4')
					LOCK1=self.rapid.get_digital_io('LOCK1')
					LOCK2=self.rapid.get_digital_io('LOCK2')

					self.sensor_list=[IDEC1,IDEC2,IDEC3,IDEC4,LOCK1,LOCK2]

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

	# print("Press ctrl+c to quit")
	# signal.sigwait([signal.SIGTERM,signal.SIGINT])
	input('press enter to quit')
	gripper_inst.StopStreaming()
	# gripper_inst.pi_pwm.stop()
