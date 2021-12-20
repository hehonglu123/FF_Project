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
fusing_sub=RRN.SubscribeService(url)
fusing_obj=fusing_sub.GetDefaultClientWait(1)
sensor_readings = fusing_sub.SubscribeWire("sensor_readings")

##############################################property check#######################################
fusing_obj.current_ply_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR HICKEY 44'
fusing_obj.current_interlining_fabric_type.fabric_name='PD19_016C-FR-LFT-LWR-INT HICKEY 44'

print(fusing_obj.current_ply_fabric_type.fabric_name)
print(fusing_obj.current_interlining_fabric_type.fabric_name)

##############################################sensor wire check############################################
now=time.time()
while time.time()-now <5:
	print(sensor_readings.InValue)
	time.sleep(0.5)


##############################################pipe check############################################
p=fusing_obj.trigger_fusing_system.Connect(-1)

p.SendPacket(trigger)  
