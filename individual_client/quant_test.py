import cv2, sys
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from temp_match import *
from fabric_detection import detection

#table ROI
ROI=np.array([[160,600],[165,1153]])	#ROI [[r1,r2],[c1,c2]]
#template size
template_size=360*1.414
#load template
temp_alg=['rgb','hsv','rgba','hsva','bw','bwa']
error_mtx=np.zeros((6,6))

for i in range(6):
	if i==3 or i==1:
		continue
	template=cv2.imread('../client_yaml/template'+str(i)+'.png',cv2.IMREAD_UNCHANGED)
	mask=np.where(template[:,:,-1]>0, 1, 0)
	#calc avg template color
	non_zeros=np.count_nonzero(template[:,:,-1])
	B=np.sum(template[:,:,0]*mask[:,:])/non_zeros
	G=np.sum(template[:,:,1]*mask[:,:])/non_zeros
	R=np.sum(template[:,:,2]*mask[:,:])/non_zeros
	avg_color=[B,G,R]

	for m in range(len(temp_alg)):

		for j in range(6):
			#load test image
			test_image=cv2.imread('image_data/temp'+str(i)+'_test'+str(j)+'.jpg',cv2.IMREAD_UNCHANGED)
			try:
				#test test image within ROI
				roi_frame=test_image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
				(orientation,centroid1)=detection(roi_frame,avg_color)
				center1=centroid1[0]+ROI[:,0]
				test_region=test_image[int(center1[0]-template_size/2):int(center1[0]+template_size/2),int(center1[1]-template_size/2):int(center1[1]+template_size/2)]

				# cv2.circle(test_image, tuple(np.flip(center1).astype(int)), 10,(0,0,255), -1)	
				# cv2.namedWindow("Image")
				# cv2.imshow("Image",test_image)
				# cv2.waitKey(0) 
				# cv2.destroyAllWindows()

				
				angle,centroid2=match_w_ori(test_region,template, orientation,temp_alg[m])
				center2=centroid2[0]+np.array([int(center1[0]-template_size/2),int(center1[1]-template_size/2)])
				error_mtx[i][m]+=np.linalg.norm(center2-center1)
			except:
				print('filtered failed','fabric type',i,'image',j)
				test_region=test_image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
				angle,center2=match(test_region,template)


error_mtx/=6
print(error_mtx)


