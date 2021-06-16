import cv2, sys
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from temp_match import match
from fabric_detection import detection

#table ROI
ROI=np.array([[160,600],[165,1153]])	#ROI [[r1,r2],[c1,c2]]
#template size
template_size=360*1.414
#load template
template=cv2.imread('../client_yaml/template0.png',cv2.IMREAD_UNCHANGED)
mask=np.where(template[:,:,-1]>0, 1, 0)
#calc avg template color
non_zeros=np.count_nonzero(template[:,:,-1])
B=np.sum(template[:,:,0]*mask[:,:])/non_zeros
G=np.sum(template[:,:,1]*mask[:,:])/non_zeros
R=np.sum(template[:,:,2]*mask[:,:])/non_zeros
avg_color=[B,G,R]


#load test image
test_image=cv2.imread('image_data/temp_test1.jpg',cv2.IMREAD_UNCHANGED)

#test test image within ROI
roi_frame=test_image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
(orientation,centroid)=detection(roi_frame,avg_color)
center=centroid[0]+ROI[:,0]
cv2.circle(test_image, tuple(np.flip(center).astype(int)), 10,(0,0,255), -1)	

# cv2.namedWindow("Image")
# cv2.imshow("Image",test_image)
# cv2.waitKey(0) 
# cv2.destroyAllWindows() 

test_region=test_image[int(center[0]-template_size/2):int(center[0]+template_size/2),int(center[1]-template_size/2):int(center[1]+template_size/2)]
cv2.namedWindow("Image")
cv2.imshow("Image",test_region)
cv2.waitKey(0) 
cv2.destroyAllWindows() 
angle,center=match(test_region,template)
# print(angle,center)
# cv2.circle(test_region, (int(center[0]),int(center[1])),10,(0,0,255), -1)		
# cv2.imshow("image", test_region)
# cv2.waitKey(0)
# cv2.destroyAllWindows()






# w, h = template.shape[::-1]
# res = cv2.matchTemplate(test_region,template,eval('cv2.TM_SQDIFF'))
# min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
# top_left=min_loc
# bottom_right = (top_left[0] + w, top_left[1] + h)
# print(top_left,bottom_right,min_val)
# cv2.rectangle(test_region,top_left, bottom_right, 255, 2)
# plt.subplot(121),plt.imshow(res,cmap = 'gray')
# plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
# plt.subplot(122),plt.imshow(test_region,cmap = 'gray')
# plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
# plt.show()