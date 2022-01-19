#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy, yaml
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
robot_name='abb'
sys.path.append('../toolbox/')
from vel_emulate_sub import EmulatedVelocityControl
from general_robotics_toolbox import *
# from cv2 import aruco
from temp_match import bold_edge
from abb_def import *

def jog_joint(q,max_v):
	global vel_ctrl
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()

	diff=q-vel_ctrl.joint_position()
	time_temp=np.linalg.norm(diff)/max_v

	qdot_temp=np.clip((1.15*diff)/time_temp,-max_v,max_v)

	while np.linalg.norm(q-vel_ctrl.joint_position())>0.01:
		
		diff=q-vel_ctrl.joint_position()
		qdot=np.where(np.abs(diff) > 0.05, qdot_temp, diff)

		vel_ctrl.set_velocity_command(qdot)


	vel_ctrl.set_velocity_command(np.zeros((6,)))
	vel_ctrl.disable_velocity_mode() 


def aruco_process(frame):
	aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
	parameters =  aruco.DetectorParameters_create()
	parameters.adaptiveThreshWinSizeMin=5
	corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
	tag_centroids=np.average(corners,axis=2)
	return tag_centroids, ids


def angle(p1, p2):
	###comparator for vertices order
    k = (p2[1] - p1[1]) / np.linalg.norm(p1- p2)

    if k >= 0:
        if p2[0] >= p1[0]: # First Quadrant
            return (2.0 * math.pi - math.asin(k))
        else: # Second Quadrant
            return (math.pi + math.asin(k))
    else:
        if p2[0] >= p1[0]: # Fourth Quadrant
            return math.asin(-k)
        else: # Third Quadrant
            return (math.pi - math.asin(-k))


def PolyArea2D(pts):
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area
###determine pixels/unit distance, ROI
def preprocess(image):
	# tag_centroids, ids = aruco_process(image)
	tag_centroids=np.array([[[ 70,25]], [[70,697]],[[1206,25]],[[1208,700]]])
	ROI=np.array([[np.min(tag_centroids[:,:,1]),np.max(tag_centroids[:,:,1])],[np.min(tag_centroids[:,:,0]),np.max(tag_centroids[:,:,0])]]).astype(int)		#[[r1,r2],[c1,c2]]
	
	vertices=np.squeeze(tag_centroids,axis=1)
	sorted_vertices=sorted(vertices, key = lambda point: -angle(point, np.average(vertices,axis=0)))
	ppu=PolyArea2D(sorted_vertices)/384000		#pixel area / plate area in mm^2
	
	return ROI,ppu


def ImageToMat(image):

    frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

    return frame2

        
def edge_detection(image):
	return cv2.Canny(image, 50, 200,apertureSize =3)
def main():
	global vel_ctrl
	###read yamls
	with open(r'../client_yaml/testbed.yaml') as file:
		testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)

	home=testbed_yaml['home']
	vision_q=testbed_yaml['vision_q']
	vision_p=testbed_yaml['vision_p']


	###realsense connect
	url='rr+tcp://localhost:59823?service=camera'
	#Startup, connect, and pull out the camera from the objref    
	cam=RRN.ConnectService(url)


	robot_sub=RRN.SubscribeService('rr+tcp://pi_fuse:58651/?service=robot')
	robot=robot_sub.GetDefaultClientWait(1)
	state_w = robot_sub.SubscribeWire("robot_state")
	cmd_w=robot_sub.SubscribeWire('position_command')
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)

	##########Initialize robot constants
	robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
	halt_mode = robot_const["RobotCommandMode"]["halt"]
	position_mode = robot_const["RobotCommandMode"]["position_command"]
	jog_mode = robot_const["RobotCommandMode"]["jog"]

	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = position_mode

	###temp, lift up
	q=state_w.InValue.joint_position
	pose=inv.fwd(q)
	# robot.jog_freespace(inv([pose.p[0],pose.p[1],0.6],pose.R), 0.3*np.ones(6), True)
	jog_joint(inv([pose.p[0],pose.p[1],0.6],pose.R), 0.3)


	##home
	# robot.jog_freespace(inv(home,R_ee(0)), 0.3*np.ones(6), True)
	jog_joint(inv(home,R_ee(0)), 0.3)


	q=inv(vision_p+np.array([0.15,0,0.15]),R_ee(0))
	# robot.jog_freespace(q, 0.3*np.ones(6), True)
	jog_joint(q, 0.3)

	# robot.jog_freespace(vision_q, 0.2*np.ones(6), True)
	jog_joint(vision_q, 0.2)
	###capture image at calib configuration and process image
	current_frame=cv2.rotate(ImageToMat(cam.capture_frame()), cv2.ROTATE_180)
	cv2.imwrite('../client_yaml/plate_calib.jpg', current_frame)
	ROI, ppu=preprocess(current_frame)
	ROI=ROI.flatten().tolist()
	print(ROI)
	###go back to normal config async
	q=inv(vision_p+np.array([0.15,0,0.15]),R_ee(0))
	# robot.jog_freespace(q, 0.3*np.ones(6), False)
	jog_joint(q,0.3)

	###edge background	
	roi_frame=cv2.cvtColor(current_frame[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)
	edged=bold_edge(edge_detection(roi_frame),num_pix=2)
	cv2.imwrite('../client_yaml/plate_edge.jpg', edged)

	vision_yaml={'ROI':ROI.flatten().tolist(),'ppu':ppu.item()}
	with open('../client_yaml/vision.yaml', 'w') as file:
		yaml.dump(vision_yaml, file)
	

def main2():
	image=cv2.imread('../client_yaml/plate_calib.jpg')
	ROI,ppu=preprocess(image)
	ROI=ROI.flatten().tolist()
	print(ROI,ppu)
	roi_frame=cv2.cvtColor(image[ROI[0]:ROI[1],ROI[2]:ROI[3]], cv2.COLOR_BGR2GRAY)
	edged=bold_edge(edge_detection(roi_frame),num_pix=2)
	cv2.imwrite('../client_yaml/plate_edge.jpg', edged)

if __name__ == '__main__':
    main2()
