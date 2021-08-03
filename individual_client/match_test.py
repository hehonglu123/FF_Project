import cv2, sys
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from temp_match import *
from fabric_detection import detection

#table ROI
ROI=np.array([[29,690],[147,1252]])	#ROI [[r1,r2],[c1,c2]]
#template size
template_size=360*1.414
#load template
template=cv2.imread('image_data/wool.png',cv2.IMREAD_UNCHANGED)
mask=np.where(template[:,:,-1]>0, 1, 0)
#calc avg template color
non_zeros=np.count_nonzero(template[:,:,-1])
B=np.sum(template[:,:,0]*mask[:,:])/non_zeros
G=np.sum(template[:,:,1]*mask[:,:])/non_zeros
R=np.sum(template[:,:,2]*mask[:,:])/non_zeros
avg_color=[B,G,R]

#load test image
test_image=cv2.imread('image_data/wool.jpg',cv2.IMREAD_UNCHANGED)

# try:
# 	#test test image within ROI
# 	roi_frame=test_image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
# 	(orientation,centroid1)=detection(roi_frame,avg_color)
# 	center1=centroid1[0]+ROI[:,0]
# 	test_region=test_image[int(center1[0]-template_size/2):int(center1[0]+template_size/2),int(center1[1]-template_size/2):int(center1[1]+template_size/2)]

# 	# cv2.circle(test_image, tuple(np.flip(center1).astype(int)), 10,(0,0,255), -1)	
# 	# cv2.namedWindow("Image")
# 	# cv2.imshow("Image",test_image)
# 	# cv2.waitKey(0) 
# 	# cv2.destroyAllWindows()

# 	temp_alg=['rgb','hsv','rgba','hsva','bw','bwa']
# 	angle,centroid2=match_w_ori(test_region,template, orientation,'edge')
# 	center2=centroid2[0]+np.array([int(center1[0]-template_size/2),int(center1[1]-template_size/2)])
# except:
# 	print('filtered failed')
# 	test_region=test_image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
# 	angle,center2=match(test_region,template,'edge')
# 	center2=np.flip(center2+ROI[:,0])
	
test_region=test_image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
angle,center2=match_w_ori(test_region,template,[0],'edge')
center2=np.flip(center2+np.flip(ROI[:,0]))


print(angle)
###show final results
cv2.circle(test_image, (int(center2[1]),int(center2[0])),10,(0,0,255), -1)		
cv2.imshow("image", test_image)
cv2.waitKey(0)
cv2.destroyAllWindows()


