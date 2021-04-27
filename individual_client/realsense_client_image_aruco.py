#Simple example Robot Raconteur webcam client
#This program will show a live streamed image from
#the webcams.  Because Python is a slow language
#the framerate is low...

from RobotRaconteur.Client import *
from cv2 import aruco

import cv2, sys, traceback, argparse, time
import numpy as np
import traceback


#Function to take the data structure returned from the Webcam service
#and convert it to an OpenCV array
def ImageToMat(image):

	if image.image_info.encoding == image_consts["ImageEncoding"]["bgr888"]:
		frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')
	elif image.image_info.encoding == image_consts["ImageEncoding"]["depth_u16"]:
		depth_img =image.data.view(dtype=np.uint16).reshape([image.image_info.height, image.image_info.width], order='C')
		frame2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.1), cv2.COLORMAP_JET)
	else:
		assert False, "Unexpected data type"
	return frame2

image_consts=None
current_frame=None
now=time.time()
#This function is called when a new pipe packet arrives
def new_frame(pipe_ep):
	global current_frame, now
	# print('fps= ', 1/(time.time()-now))
	now=time.time()
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame=ImageToMat(image)

		return

def aruco_process(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        # parameters.minMarkerPerimeterRate=0.00001
        # parameters.adaptiveThreshConstant=20
        # # parameters.minMarkerDistanceRate=0.005
        # parameters.adaptiveThreshWinSizeMin=5
        # parameters.adaptiveThreshWinSizeMax=10
        # parameters.adaptiveThreshWinSizeStep=1

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        return frame_markers

def main():
	#Accept the names of the webcams and the nodename from command line
	parser = argparse.ArgumentParser(description="RR plug and play client")
	parser.add_argument("--type",type=str,default='rgb',help="type of image")
	args, _ = parser.parse_known_args()

	cam_dict={'rgb':0,'depth':1}

	url='rr+tcp://localhost:25415?service=Multi_Cam_Service'

	#Startup, connect, and pull out the camera from the objref    
	Multi_Cam_obj=RRN.ConnectService(url)

	global image_consts
	image_consts = RRN.GetConstants('com.robotraconteur.image', Multi_Cam_obj)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	cam=Multi_Cam_obj.get_cameras(cam_dict[args.type])

	# Use `captured_img = cam.capture_frame()` to synchronously capture an image

	p=cam.frame_stream.Connect(-1)

	#Set the callback for when a new pipe packet is received to the
	#new_frame function
	p.PacketReceivedEvent+=new_frame


	try:
		cam.start_streaming()
	except: 
		traceback.print_exc()
		pass

	cv2.namedWindow("Image")

	while True:
		#Just loop resetting the frame
		#This is not ideal but good enough for demonstration

		if (not current_frame is None):

			cv2.imshow("Image",aruco_process(current_frame))
		if cv2.waitKey(50)!=-1:
			break
	cv2.destroyAllWindows()

	p.Close()
	cam.stop_streaming()



if __name__ == '__main__':
	main()
