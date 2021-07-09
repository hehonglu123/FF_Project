import numpy as np
import sys, cv2, yaml
sys.path.append('toolbox/')
from temp_match import match, match_w_ori
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
ROI=np.array([[160,600],[165,1153]])	#ROI [[r1,r2],[c1,c2]]

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


###fabric template
template=cv2.imread('client_yaml/template3.png',cv2.IMREAD_UNCHANGED)
mask=np.where(template[:,:,-1]>0, 1, 0)
#calc avg template color
non_zeros=np.count_nonzero(template[:,:,-1])
B=np.sum(template[:,:,0]*mask[:,:])/non_zeros
G=np.sum(template[:,:,1]*mask[:,:])/non_zeros
R=np.sum(template[:,:,2]*mask[:,:])/non_zeros
avg_color=[B,G,R]

while True:

	#Just loop resetting the frame
	#This is not ideal but good enough for demonstration
	if (not current_frame is None):
		roi_frame=current_frame[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]

		(orientation,centroid)=detection(roi_frame,avg_color)

		try:
			center=centroid[0]+ROI[:,0]
			# center+=[offset_white_left[0]*np.cos(orientation_white[0])-offset_white_left[1]*np.sin(orientation_white[0]),offset_white_left[0]*np.sin(orientation_white[0])+offset_white_left[1]*np.cos(orientation_white[0])]
			center=center.astype(int)
			
			# angle,center_temp=match(current_frame[center[0]-300:center[0]+300,center[1]-300:center[1]+300,:],template_left)
			angle,center_temp=match_w_ori(current_frame[center[0]-300:center[0]+300,center[1]-300:center[1]+300,:],template,orientation,'edge')
			center=(center[0]-300+center_temp[0],center[1]-300+center_temp[1])

			
		except:
			angle,center_temp=match(roi_frame,template,'edge')
			center=list(center_temp[::-1])+ROI[:,0]
			# continue
		
		p=pixel2coord2(R_realsense,p_realsense,np.flip(center).astype(float),0)
		#draw dots	
		cv2.circle(current_frame, tuple(np.flip(center).astype(int)), 10,(0,0,255), -1)		
		current_frame = cv2.putText(current_frame, str(p[0])+','+str(p[1])+' ,'+str(angle), org = tuple(np.flip(center).astype(int)), 
               fontScale = 1, fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL,color = (255, 0, 0), thickness = 2, lineType=cv2.LINE_AA)


		current_frame = cv2.rectangle(current_frame, (ROI[1][0],ROI[0][0]), (ROI[1][1],ROI[0][1]), color = (255, 0, 0), thickness=2)


		cv2.imshow("Image",current_frame)
	if cv2.waitKey(50)!=-1:
		break
cv2.destroyAllWindows()