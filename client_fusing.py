from RobotRaconteur.Client import *
import RobotRaconteurCompanion as RRC
import sys, time, yaml, traceback
import numpy as np
from importlib import import_module
sys.path.append('../toolbox/')
from general_robotics_toolbox import *    

############################register RR params#############################################
RRC.RegisterStdRobDefServiceTypes(RRN)
RRN.RegisterServiceTypeFromFile("edu.rpi.robotics.fusing_system")
trigger=RRN.NewStructure("edu.rpi.robotics.fusing_system.FusingOperationTrigger")
trigger.number_of_operations=1

fusing_laptop='192.168.51.188'
robosewclient='192.168.51.61'
pi_fuse='192.168.51.25'
my_laptop='192.168.51.181'

url='rr+tcp://rpi:12180/?service=fusing_service'
# fusing_sub=RRN.SubscribeService(url)
# fusing_obj=fusing_sub.GetDefaultClientWait(1)
fusing_obj=RRN.ConnectService(url)

##############################################property check#######################################
print(fusing_obj.current_ply_fabric_type.fabric_name)
print(fusing_obj.current_interlining_fabric_type.fabric_name)

##############################################pipe check############################################
p=fusing_obj.trigger_fusing_system.Connect(-1)
while True:
	p.SendPacket(trigger)  
	print('sent')
	time.sleep(1)