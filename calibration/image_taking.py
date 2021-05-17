import cv2, time
from RobotRaconteur.Client import *
import numpy as np

current_frame_rgb=None

def ImageToMat(image):

	frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

	return frame2

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


url='rr+tcp://localhost:25415?service=Multi_Cam_Service'

#Startup, connect, and pull out the camera from the objref    
Multi_Cam_obj=RRN.ConnectService(url)

#Connect the pipe FrameStream to get the PipeEndpoint p
cam_rgb=Multi_Cam_obj.get_cameras(0)
p_rgb=cam_rgb.frame_stream.Connect(-1)

p_rgb.PacketReceivedEvent+=new_frame_rgb
try:
	cam_rgb.start_streaming()
except: 
	traceback.print_exc()
	pass

#wait till valid image
while current_frame_rgb is None:
	continue
print('started')
for i in range(50):
	time.sleep(3)
	cv2.imwrite('intrinsic_calibration/chessboard/'+str(i)+'.jpg',current_frame_rgb)
	



# cv2.imshow("image", frame)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
