#!/usr/bin/python3
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteur.RobotRaconteurPythonError import StopIterationException
import sys, os, time, argparse, traceback, cv2, copy, yaml, threading
import numpy as np



class fusing_pi(object):
	def __init__(self):
		self.current_ply_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
		self.current_interlining_fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
		self.error_message_type=RRN.NewStructure("com.robotraconteur.eventlog.EventLogMessage")
		self.finish_signal_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FinishSignal")
		self.current_errors=[]

		self.current_ply_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR HICKEY 36'
		self.current_interlining_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR-INT HICKEY 36'

		self.current_operation_count=0

		self.actuator_position={'bin1':None,'bin2':None,'robot':None}
		##################################sensor background threading#######################################
		self._streaming=False
		self._lock = threading.Lock()

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
				print(traceback.format_exc())

	#sensor reading
	def threadfunc(self):
		while(self._streaming):
			time.sleep(0.2)
			with self._lock:
				try:
					self.sensor_readings.OutValue=np.zeros(6,dtype=bool)
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

	def actuate(self,position,action):
		try:
			self.actuator_position[position]
			print('set '+position+' to '+str(action))
		except:
			self.trigger_error('actuator wrong',traceback.format_exc())

	def initialize(self):
		print('initializing')
		return True
	###ESTOP
	def stop_fusing(self):
		self.trigger_error('Operation Error: ', 'Stopped by Operator')

	def execute(self,stacks):
		for i in range(stacks):
			time.sleep(1)
			self.current_operation_count+=1

		###send finish signal
		self.finish_signal_type.current_errors=[]
		self.finish_signal_type.finished=True
		self.finish_signal.SendPacket(self.finish_signal_type)

	###error handling
	def trigger_error(self,title,error_msg):
		self.error_message_type.title=title
		self.error_message_type.message=error_msg
		self.finish_signal_type.current_errors=[self.error_message_type]
		self.finish_signal_type.finished=True

		print(error_msg)
		###send stop signal to WGC
		self.finish_signal.SendPacket(self.finish_signal_type)
		



def main():
	RRC.RegisterStdRobDefServiceTypes(RRN)


	with RR.ServerNodeSetup("fusing_service",12180) as node_setup:
		
		RRN.RegisterServiceTypeFromFile("edu.rpi.robotics.fusing_system")

		fusing_pi_obj=fusing_pi()
		fusing_pi_obj.StartStreaming()

		service_ctx = RRN.RegisterService("fusing_service","edu.rpi.robotics.fusing_system.FusingSystem",fusing_pi_obj)


		input('press enter to quit')

		fusing_pi_obj.StopStreaming()


if __name__ == '__main__':
	main()
