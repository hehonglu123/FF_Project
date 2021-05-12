#!/usr/bin/env python3
import numpy as np
from RobotRaconteur.Client import *
import sys, time, yaml, argparse
from scipy.optimize import leastsq
from importlib import import_module
from cv2 import aruco
import cv2
sys.path.append('../toolbox')
from general_robotics_toolbox import Robot, fwdkin
from vel_emulate_sub import EmulatedVelocityControl

from pixel2coord import convert
from autodiscovery import autodiscover
from jog_joint import jog_joint

current_frame_rgb=None
current_frame_depth=None
new_val=False
R_realsense=None
p_realsense=None
def ImageToMat(image):

	frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

	return frame2

#This function is called when a new pipe packet arrives
def new_frame_rgb(pipe_ep):
	global current_frame_rgb, new_val
	print('received')
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):

		#Receive the packet
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame_rgb=ImageToMat(image)
		new_val=True

		return
def new_frame_depth(pipe_ep):
	global current_frame_depth
	print('received2')
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		depth_data=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame_depth=depth_data.data.view(dtype=np.uint16).reshape([depth_data.image_info.height, depth_data.image_info.width], order='C')

		return




def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))



url='rr+tcp://localhost:25415?service=Multi_Cam_Service'

#Startup, connect, and pull out the camera from the objref    
Multi_Cam_obj=RRN.ConnectService(url)

#Connect the pipe FrameStream to get the PipeEndpoint p
cam_rgb=Multi_Cam_obj.get_cameras(0)
cam_depth=Multi_Cam_obj.get_cameras(1)


p_rgb=cam_rgb.frame_stream.Connect(-1)
p_depth=cam_depth.frame_stream.Connect(-1)
#Set the callback for when a new pipe packet is received to the
#new_frame function
p_rgb.PacketReceivedEvent+=new_frame_rgb
p_depth.PacketReceivedEvent+=new_frame_depth

try:
	cam_rgb.start_streaming()
	cam_depth.start_streaming()
except: 
	traceback.print_exc()
	pass


while True:

	if (not current_frame_rgb is None):

		cv2.imshow("Image",current_frame_rgb)
	if cv2.waitKey(50)!=-1:
		break
cv2.destroyAllWindows()

# p_rgb.close()
# p_depth.close()
cam_rgb.stop_streaming()
cam_depth.stop_streaming()