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
		self.current_errors=[]

		self.current_ply_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR HICKEY 36'
		self.current_interlining_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR-INT HICKEY 36'

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
				print('executing '+str(dat.number_of_operations))
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


	###ESTOP
	def stop_fusing(self):
		print('stop')


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