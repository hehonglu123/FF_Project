#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy, yaml
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('../toolbox/')
from general_robotics_toolbox import *
from cv2 import aruco



inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)

def aruco_process(frame):
	aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
	parameters =  aruco.DetectorParameters_create()
	parameters.adaptiveThreshWinSizeMin=5
	corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
	tag_centroids=np.average(corners,axis=2)
	return tag_centroids, ids

def PolyArea2D(pts):
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area
###determine pixels/unit distance, ROI
def preprocess(image):
	tag_centroids, ids = aruco_process(image)
	ROI=np.array([[np.min(tag_centroids[:,:,1]),np.max(tag_centroids[:,:,1])],[np.min(tag_centroids[:,:,0]),np.max(tag_centroids[:,:,0])]]).astype(int)		#[[r1,r2],[c1,c2]]
	ppu=PolyArea2D(np.squeeze(tag_centroids,axis=1))/358800		#pixel area / plate area in mm
	
	return ROI,ppu


def ImageToMat(image):

    frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

    return frame2
current_frame=None

def new_frame(pipe_ep):
    global current_frame, now
    # print('fps= ', 1/(time.time()-now))
    now=time.time()
    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        
        image=pipe_ep.ReceivePacket()
        #Convert the packet to an image and set the global variable
        current_frame=cv2.rotate(ImageToMat(image), cv2.ROTATE_180)

        return
        
def main():

	###read yamls
	with open(r'testbed.yaml') as file:
		testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)

	home=testbed_yaml['home']
	vision_q=testbed_yaml['vision_q']
	vision_p=testbed_yaml['vision_p']


	###realsense connect
	url='rr+tcp://localhost:59823?service=camera'
	#Startup, connect, and pull out the camera from the objref    
	c=RRN.ConnectService(url)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	p=c.frame_stream.Connect(-1)

	#Set the callback for when a new pipe packet is received to the
	#new_frame function
	p.PacketReceivedEvent+=new_frame
	try:
	    c.start_streaming()
	except: 
	    traceback.print_exc()
	    pass


	robot_sub=RRN.SubscribeService('rr+tcp://[fe80::16ff:3758:dcde:4e15]:58651/?nodeid=16a22280-7458-4ce9-bd4d-29b55782a2e1&service=robot')
	robot=robot_sub.GetDefaultClientWait(1)
	state_w = robot_sub.SubscribeWire("robot_state")


	##########Initialize robot constants
	robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
	halt_mode = robot_const["RobotCommandMode"]["halt"]
	position_mode = robot_const["RobotCommandMode"]["position_command"]
	jog_mode = robot_const["RobotCommandMode"]["jog"]

	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = jog_mode

	###temp, lift up
	q=state_w.InValue.joint_position
	pose=inv.fwd(q)
	robot.jog_freespace(inv.inv([pose.p[0],pose.p[1],0.6],pose.R), 0.3*np.ones(6), True)



	##home
	robot.jog_freespace(inv.inv(home,R_ee.R_ee(0)), 0.3*np.ones(6), True)


	global current_frame

	q=inv.inv(vision_p+np.array([0.15,0,0.15]),R_ee.R_ee(0))
	robot.jog_freespace(q, 0.3*np.ones(6), True)

	robot.jog_freespace(vision_q, 0.2*np.ones(6), True)

	cv2.imwrite('plate_calib.jpg', current_frame)
	ROI, ppu=preprocess(current_frame)


	q=inv.inv(vision_p+np.array([0.15,0,0.15]),R_ee.R_ee(0))
	robot.jog_freespace(q, 0.3*np.ones(6), False)

	vision_yaml={'ROI':ROI.flatten().tolist(),'ppu':ppu.item()}
	with open('vision.yaml', 'w') as file:
		yaml.dump(vision_yaml, file)
	

def main2():
	image=cv2.imread('plate_calib.jpg')
	preprocess(image)

if __name__ == '__main__':
    main()
