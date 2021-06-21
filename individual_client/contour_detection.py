import cv2, sys
import numpy as np
sys.path.append('../toolbox/')
from temp_match import *
from matplotlib import pyplot as plt


def contour(img):
	img_grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	#set a thresh
	thresh = 100
	#get threshold image
	ret,thresh_img = cv2.threshold(img_grey, thresh, 255, cv2.THRESH_BINARY)
	#find contours
	contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	# for cnt in contours:
	#     approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
	#     print (len(approx))


	#create an empty image for contours
	img_contours = np.zeros(img.shape)
	# draw the contours on the empty image
	cv2.drawContours(img_contours, contours, -1, (0,255,0), 3)
	return img_contours


test_image = cv2.imread('image_data/temp2_test0.jpg')
template = cv2.imread('../client_yaml/template2.png')

ROI=np.array([[160,600],[165,1153]])	#ROI [[r1,r2],[c1,c2]]
roi_frame=test_image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]

test_imgage_contour=contour(roi_frame).astype(np.uint8)
template_contour=contour(template).astype(np.uint8)
template_contour=template_contour[3:-3,3:-3,:]
####################################
# min_val, min_loc=contour_temp_match(test_imgage_contour,template_contour)
# print(min_loc)



###############################################

angle,center2=match(test_imgage_contour,template_contour,'contour')
print(center2,angle)


##############################################
# res = cv2.matchTemplate(test_imgage_contour, template_contour, cv2.TM_SQDIFF)
# min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)


# w, h = template_contour.shape[1],template_contour.shape[0]

# top_left=min_loc
# bottom_right = (top_left[0] + w, top_left[1] + h)
# # template_rt=rotate_image(template_contour,angle,[0,0,0])


# cv2.rectangle(test_imgage_contour,top_left, bottom_right, 255, 2)
# plt.subplot(121),plt.imshow(res,cmap = 'gray')
# plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
# plt.subplot(122),plt.imshow(test_imgage_contour,cmap = 'gray')
# plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
# plt.show()



cv2.imshow("image", test_imgage_contour)
cv2.waitKey(0)
cv2.destroyAllWindows()