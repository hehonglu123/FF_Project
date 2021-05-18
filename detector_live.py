import numpy as np
import sys, cv2, yaml
sys.path.append('toolbox/')

from pixel2coord import *

from RobotRaconteur.Client import *
sys.path.append('individual_client/')
from fabric_detection import detection

def ImageToMat(image):
	global image_consts
	if image.image_info.encoding == image_consts["ImageEncoding"]["bgr888"]:
		frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')
	elif image.image_info.encoding == image_consts["ImageEncoding"]["depth_u16"]:
		depth_img =image.data.view(dtype=np.uint16).reshape([image.image_info.height, image.image_info.width], order='C')
		frame2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.1), cv2.COLORMAP_JET)
	else:
		assert False, "Unexpected data type"
	return frame2

current_frame=None
#This function is called when a new pipe packet arrives
def new_frame(pipe_ep):
	global current_frame
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame=ImageToMat(image)

		return


#Startup, connect, and pull out the camera from the objref  
url='rr+tcp://localhost:25415?service=Multi_Cam_Service'  
Multi_Cam_obj=RRN.ConnectService(url)
image_consts = RRN.GetConstants('com.robotraconteur.image', Multi_Cam_obj)

#Connect the pipe FrameStream to get the PipeEndpoint p
rgb_cam=Multi_Cam_obj.get_cameras(0)
p=rgb_cam.frame_stream.Connect(-1)
#Set the callback for when a new pipe packet is received to the
#new_frame function
p.PacketReceivedEvent+=new_frame
try:
	rgb_cam.start_streaming()
except: 
	traceback.print_exc()
	pass

cv2.namedWindow("Image")

with open('calibration/camera_extrinsic.yaml') as file:
	realsense_param = yaml.load(file, Loader=yaml.FullLoader)
p_realsense=np.array(realsense_param['p'])
R_realsense=np.array(realsense_param['R'])

while True:
	#Just loop resetting the frame
	#This is not ideal but good enough for demonstration
	if (not current_frame is None):
		(orientation,centroid)=detection(current_frame,[30,51,1])
		(orientation2,centroid2)=detection(current_frame,[112,55,0])
		try:
			centroid[0]
		except:
			continue
		p=pixel2coord2(R_realsense,p_realsense,np.flip(centroid[0]),0)
		#draw dots
		cv2.circle(current_frame, tuple(np.flip(centroid[0]).astype(int)), 10,(0,0,255), -1)		
		current_frame = cv2.putText(current_frame, str(p[0])+','+str(p[1])+str(orientation), org = tuple(np.flip(centroid[0]).astype(int)), 
               fontScale = 1, fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL,color = (255, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
		

		
		try:
			centroid2[0]
		except:
			continue
		p=pixel2coord2(R_realsense,p_realsense,np.flip(centroid2[0]),0)
		#draw dots
		cv2.circle(current_frame, tuple(np.flip(centroid2[0]).astype(int)), 10,(0,0,255), -1)		
		current_frame = cv2.putText(current_frame, str(p[0])+','+str(p[1])+str(orientation2), org = tuple(np.flip(centroid2[0]).astype(int)), 
               fontScale = 1, fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL,color = (255, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
		


		cv2.imshow("Image",current_frame)
	if cv2.waitKey(50)!=-1:
		break
cv2.destroyAllWindows()