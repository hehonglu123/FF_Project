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


fusing_laptop='192.168.51.116'
robosewclient='192.168.51.61'
pi_fuse='192.168.51.25'
my_laptop='192.168.51.181'

url='rr+tcp://'+fusing_laptop+':12180/?service=fusing_service'
fusing_sub=RRN.SubscribeService(url)
fusing_obj=fusing_sub.GetDefaultClientWait(1)
sensor_readings = fusing_sub.SubscribeWire("sensor_readings")

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

print(fusing_obj.initialize())
while True:
    time.sleep(0.5)