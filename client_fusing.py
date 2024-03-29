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

def finish_trigger_cb(pipe_ep):
    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        finish_signal=pipe_ep.ReceivePacket()

        if finish_signal.finished and len(finish_signal.current_errors)==0:
            print('finished successfully')
        else:
            i=0
            for error in finish_signal.current_errors:
                print('Error '+str(i)+': '+error.title+' '+error.message)
                i+=1



############################register RR params#############################################
RRC.RegisterStdRobDefServiceTypes(RRN)
RRN.RegisterServiceTypeFromFile("edu.rpi.robotics.fusing_system")


fusing_laptop='192.168.51.116'
robosewclient='192.168.51.61'
pi_fuse='192.168.51.25'
my_laptop='192.168.51.181'

url='rr+tcp://'+robosewclient+':12180/?service=fusing_service'
fusing_sub=RRN.SubscribeService(url)
fusing_sub.ClientConnectFailed += connect_failed

fusing_obj=fusing_sub.GetDefaultClientWait(1)
sensor_readings = fusing_sub.SubscribeWire("sensor_readings")

##############################################Error pipe #######################################

p=fusing_obj.finish_signal.Connect(-1)
p.PacketReceivedEvent+=finish_trigger_cb

##############################################initialization check#######################################
res=fusing_obj.initialize()
print('initialization '+str(res))

##############################################property check#######################################
print(fusing_obj.current_ply_fabric_type.fabric_name)
print(fusing_obj.current_interlining_fabric_type.fabric_name)

fabric_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
interlining_type=RRN.NewStructure("edu.rpi.robotics.fusing_system.FabricInfo")
fabric_type.fabric_name='PD19_016C-FR-LFT-UP HICKEY 56'
fabric_type.fabric_type='Army Green'
interlining_type.fabric_name='PD19_016C-FR-LFT-UP-INT HICKEY 56'
interlining_type.fabric_type='Army Black'


fusing_obj.current_ply_fabric_type=fabric_type
fusing_obj.current_interlining_fabric_type=interlining_type

print(fusing_obj.current_ply_fabric_type.fabric_name)
print(fusing_obj.current_interlining_fabric_type.fabric_name)

##############################################sensor wire check############################################
# time.sleep(0.5)
# now=time.time()
# while time.time()-now <1:
# 	print(sensor_readings.InValue)
# 	time.sleep(0.5)
# ##############################################actuator check#######################################
# print('testing actuator')
# fusing_obj.actuate('bin1',True)
# time.sleep(0.5)
# fusing_obj.actuate('bin1',False)
# time.sleep(0.5)
# fusing_obj.actuate('bin2',True)
# time.sleep(0.5)
# fusing_obj.actuate('bin2',False)
# time.sleep(0.5)
# fusing_obj.actuate('robot_lock',True)
# time.sleep(0.5)
# fusing_obj.actuate('robot_lock',False)

##############################################trigger pipe check############################################
p=fusing_obj.trigger_fusing_system.Connect(-1)

p.SendPacket(7)  

##############################################execution check############################################

while True:
    print ('current count: ',fusing_obj.current_operation_count)
    time.sleep(0.1)