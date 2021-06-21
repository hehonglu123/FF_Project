#Simple example Robot Raconteur webcam client
#This program will show a live streamed image from
#the webcams.  Because Python is a slow language
#the framerate is low...

from RobotRaconteur.Client import *

import cv2, sys, traceback, argparse
import numpy as np


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

def main():
	#Accept the names of the webcams and the nodename from command line
	parser = argparse.ArgumentParser(description="RR plug and play client")

	url='rr+tcp://localhost:25415?service=Multi_Cam_Service'

	#Startup, connect, and pull out the camera from the objref    
	Multi_Cam_obj=RRN.ConnectService(url)

	global image_consts
	image_consts = RRN.GetConstants('com.robotraconteur.image', Multi_Cam_obj)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	cam=Multi_Cam_obj.get_cameras(0)
	current_frame=ImageToMat(cam.capture_frame())
	cv2.imwrite('image_data/temp1_test11.jpg',current_frame)
	#### Depth
	# cam=Multi_Cam_obj.get_cameras(1)
	# depth_data=cam.capture_frame()
	# current_frame=ImageToMat(depth_data)
	# cv2.imwrite('image_data/depth0.jpg',current_frame)

	# np.save('image_data/depth0.npy', depth_data.data.view(dtype=np.uint16).reshape([depth_data.image_info.height, depth_data.image_info.width], order='C'))
	# np.savetxt('image_data/depth5.csv', depth_data.data.view(dtype=np.uint16).reshape([depth_data.image_info.height, depth_data.image_info.width], order='C'), delimiter=',')



if __name__ == '__main__':
	main()
