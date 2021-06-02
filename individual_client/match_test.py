import cv2, sys
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from temp_match import match

im_pth = "image_data/white1.jpg"
# im_pth = "image_data/test.jpg"

image = cv2.imread(im_pth)#,cv2.IMREAD_GRAYSCALE)
template=cv2.imread('../client_yaml/piece0_right.png',cv2.IMREAD_UNCHANGED)


# test_region=image[350:652,250:500]
test_region=image[300:650,700:1050]
# test_region=image[80:305,:]

angle,center=match(test_region,template)
print(angle,center)
cv2.circle(test_region, (int(center[0]),int(center[1])),10,(0,0,255), -1)		
cv2.imshow("image", test_region)
cv2.waitKey(0)
cv2.destroyAllWindows()






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