from RobotRaconteur.Client import *
import RobotRaconteurCompanion as RRC
import sys, time, yaml, traceback
import numpy as np
from importlib import import_module
sys.path.append('../toolbox/')
from general_robotics_toolbox import *    

###connection failed callback
def connect_failed(s, client_id, url, err):
	error_msg="Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err)
	print('Reconnecting')
	fusing_obj=fusing_sub.GetDefaultClientWait(1)



############################register RR params#############################################
RRC.RegisterStdRobDefServiceTypes(RRN)
RRN.RegisterServiceTypeFromFile("edu.rpi.robotics.fusing_system")


fusing_laptop='192.168.51.188'
robosewclient='192.168.51.61'
pi_fuse='192.168.51.25'
my_laptop='192.168.51.181'

url='rr+tcp://rpi:12180/?service=fusing_service'
fusing_sub=RRN.SubscribeService(url)
fusing_sub.ClientConnectFailed += connect_failed

fusing_obj=fusing_sub.GetDefaultClientWait(1)
sensor_readings = fusing_sub.SubscribeWire("sensor_readings")

##############################################property check#######################################
print(fusing_obj.current_ply_fabric_type.fabric_name)
print(fusing_obj.current_interlining_fabric_type.fabric_name)

fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
interlining_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
fabric_type.fabric_name='PD19_016C-FR-LFT-LWR HICKEY 44'
interlining_type.fabric_name='PD19_016C-FR-LFT-LWR-INT HICKEY 44'

fusing_obj.current_ply_fabric_type=fabric_type
fusing_obj.current_interlining_fabric_type=interlining_type

print(fusing_obj.current_ply_fabric_type.fabric_name)
print(fusing_obj.current_interlining_fabric_type.fabric_name)

##############################################sensor wire check############################################
time.sleep(0.5)
now=time.time()
while time.time()-now <1:
	print(sensor_readings.InValue)
	time.sleep(0.5)
##############################################actuator check#######################################
print('testing actuator')
fusing_obj.actuate('bin1',True)
time.sleep(0.5)
fusing_obj.actuate('bin1',False)
time.sleep(0.5)
fusing_obj.actuate('bin2',True)
time.sleep(0.5)
fusing_obj.actuate('bin2',False)
time.sleep(0.5)
fusing_obj.actuate('robot',True)
time.sleep(0.5)
fusing_obj.actuate('robot',False)

##############################################trigger pipe check############################################
p=fusing_obj.trigger_fusing_system.Connect(-1)

p.SendPacket(1)  
# p.SendPacket(1)  
# p.SendPacket(1)  
# p.SendPacket(1)  
# p.SendPacket(1)  

##############################################error check############################################
def finish_trigger_cb(pipe_ep):
    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        finish_signal=pipe_ep.ReceivePacket()

        if finish_signal.finished and len(finish_signal.current_errors)==0:
        	print('finished')
        else:
        	i=0
        	for error in finish_signal.current_errors:
        		print('Error '+str(i)+': '+error.title+' '+error.message)
        		i+=1

p=fusing_obj.finish_signal.Connect(-1)
p.PacketReceivedEvent+=finish_trigger_cb

fusing_obj.actuate('operator',True)

time.sleep(5)