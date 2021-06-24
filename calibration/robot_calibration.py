import numpy as np
from RobotRaconteur.Client import *
import sys, time, yaml, argparse, traceback
from scipy.optimize import leastsq
from importlib import import_module
from cv2 import aruco
import cv2
sys.path.append('../toolbox')
from general_robotics_toolbox import Robot, fwdkin
from vel_emulate_sub import EmulatedVelocityControl

from pixel2coord import pixel2coord2
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


def aruco_process(frame):
	global current_frame_rgb
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	parameters =  aruco.DetectorParameters_create()
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	idx=np.where(ids==169)
	if len(idx[0])==0:
		return False, None
	else:
		center=np.mean(np.array(corners[idx[0][0]][0]),axis=0)
		cv2.circle(current_frame_rgb, tuple(center.astype(int)), 10,(0,0,255), -1)
		return True, center

def calc_coord(rgb_frame,depth_frame):
	global R_realsense, p_realsense
	rtval, center=aruco_process(rgb_frame)
	if rtval:
		r=int(center[1])
		c=int(center[0])
		z=(p_realsense[-1]-depth_frame[r][c]/1000)[0]

		coord=pixel2coord2(R_realsense,p_realsense,(c,r),z)
		# print(coord[:2].tolist(),z)
		return rtval, coord[:2].tolist()
	else:
		return rtval, None

#This function is called when a new pipe packet arrives
def new_frame_rgb(pipe_ep):
	global current_frame_rgb, new_val
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
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		depth_data=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame_depth=depth_data.data.view(dtype=np.uint16).reshape([depth_data.image_info.height, depth_data.image_info.width], order='C')

		return


def my_func(x,obj,ref):

	R=np.array([[np.cos(x[0]),-np.sin(x[0])],[np.sin(x[0]),np.cos(x[0])]])
	result=np.dot(R,ref)-obj+np.array([[x[1]],[x[2]]])
	return result.flatten()

def calibrate(obj,ref): 

	result,r = leastsq(func=my_func,x0=[0,0,0],args=(np.transpose(np.array(obj)).astype(float),np.transpose(np.array(ref))))
	H=np.zeros((4,4))
	H[0][0]=np.cos(result[0])
	H[0][1]=-np.sin(result[0])
	H[1][0]=np.sin(result[0])
	H[1][1]=np.cos(result[0])
	H[2][2]=1
	H[0][-1]=result[1]
	H[1][-1]=result[2]
	H[-1][-1]=1
	return H




def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

def main():
	global new_val, p_realsense, R_realsense
	#Accept the names of the webcams and the nodename from command line
	parser = argparse.ArgumentParser(description="RR plug and play client")
	parser.add_argument("--robot-name",type=str,default='abb',help="List of camera names separated with commas")
	parser.add_argument("--url",type=str,default='rr+tcp://[fe80::922f:c9e6:5fe5:51d1]:52222/?nodeid=87518815-d3a3-4e33-a1be-13325da2461f&service=cognex',
	help="List of camera names separated with commas")
	args, _ = parser.parse_known_args()

	robot_name=args.robot_name
	url=args.url

	#########read in yaml file for robot client
	with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
		robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
	with open('camera_extrinsic.yaml') as file:
		realsense_param = yaml.load(file, Loader=yaml.FullLoader)



	###realsense part
	p_realsense=np.array(realsense_param['p'])
	R_realsense=np.array(realsense_param['R'])

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


	###robot part
	robot_sub=RRN.SubscribeService(robot_yaml['url'])
	####get client object
	robot=robot_sub.GetDefaultClientWait(1)
	####get subscription wire

	##robot wire
	cmd_w = robot_sub.SubscribeWire("position_command")
	state_w = robot_sub.SubscribeWire("robot_state")
	####connection fail callback
	robot_sub.ClientConnectFailed += connect_failed



	##########Initialize robot constants
	robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
	halt_mode = robot_const["RobotCommandMode"]["halt"]
	jog_mode = robot_const["RobotCommandMode"]["jog"]   
	position_mode = robot_const["RobotCommandMode"]["position_command"]
	robot.command_mode = halt_mode

	##########Initialize robot parameters   #need modify
	num_joints=len(robot.robot_info.joint_info)
	P=np.array(robot.robot_info.chains[0].P.tolist())
	length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
	H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))

	P[-1]+=robot_yaml['tag_position']
	# if np.linalg.norm(P[-1])!=0.:
	# 	P[-1]+=robot_yaml['tag_position']
	# else:
	# 	P[-2]+=robot_yaml['tag_position']

	robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


	#######move to start point
	print("moving to start point")

	start_joints=robot_yaml['calibration_q']


	robot.command_mode = halt_mode 
	robot.command_mode = position_mode 
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
	jog_joint(robot,vel_ctrl,start_joints,5)


	#enable velocity mode
	vel_ctrl.enable_velocity_mode()

	#initialize coordinate list
	joint_angles=[]
	cam_coordinates=[]

	print("calibrating")
	timestamp=None
	now=time.time()
	while time.time()-now<60:
		qdot=[robot_yaml['calibration_speed']]+[0]*(num_joints-1)
		vel_ctrl.set_velocity_command(np.array(qdot))

		#show image
		if (not current_frame_rgb is None):
			cv2.imshow("Image",current_frame_rgb)
		if cv2.waitKey(50)!=-1:
			break
		if new_val:
			rtval,world_coord=calc_coord(current_frame_rgb,current_frame_depth)
			if not rtval:
				new_val=False
				continue

			joint_angles.append(state_w.InValue.joint_position)
			cam_coordinates.append(world_coord)
			
		
	vel_ctrl.set_velocity_command(np.zeros((num_joints,)))
	vel_ctrl.disable_velocity_mode() 
	cv2.destroyAllWindows()
	#process data
	eef=[]
	num_samples=len(cam_coordinates)
	print("num samples collected: ",num_samples)
	for i in range(num_samples):
		transform=fwdkin(robot_def,joint_angles[i])
		p=transform.p
		eef.append(p.tolist()[:2])
	H=calibrate(cam_coordinates, eef)
	H[2][-1]=robot_yaml['height']
	print(H)
	dict_file={'H':H.tolist()}

	with open('/home/rpi/FF_Project/calibration/'+robot_name+'.yaml', 'w') as file:
		yaml.dump(dict_file, file)

	cv2.namedWindow("Image")

	# while True:
	# 	#Just loop resetting the frame
	# 	#This is not ideal but good enough for demonstration

	# 	if (not current_frame_rgb is None):

	# 		cv2.imshow("Image",current_frame_rgb)
	# 	if cv2.waitKey(50)!=-1:
	# 		break
	# cv2.destroyAllWindows()

	p_rgb.Close()
	p_depth.Close()
	cam_rgb.stop_streaming()
	cam_depth.stop_streaming()



if __name__ == '__main__':
	main()
