import cv2
import numpy as np
import sys



template_path ='../client_yaml/piece0_temp.png'
template = cv2.imread(template_path, cv2.IMREAD_UNCHANGED)
channels = cv2.split(template)
zero_channel = np.zeros_like(channels[0])
mask = np.array(channels[3])

image_path = "image_data/white0.jpg"
image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

# mask[channels[3] == 0] = 255
# mask[channels[3] == 255] = 0

# transparent_mask = None
# According to http://www.devsplanet.com/question/35658323, we can only use
# cv2.TM_SQDIFF or cv2.TM_CCORR_NORMED
# All methods can be seen here:
# http://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/template_matching/template_matching.html#which-are-the-matching-methods-available-in-opencv
method = cv2.TM_SQDIFF  # R(x,y) = \sum _{x',y'} (T(x',y')-I(x+x',y+y'))^2 (essentially, sum of squared differences)

cv2.imshow("image", cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))
cv2.imshow("template", mask)
cv2.waitKey(0)
result = cv2.matchTemplate(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY), cv2.cvtColor(template, cv2.COLOR_RGBA2GRAY), method, mask=mask)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
print (min_loc,min_val,max_loc)
