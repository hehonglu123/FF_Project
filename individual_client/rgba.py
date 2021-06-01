import cv2, sys
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from temp_match import match

im_pth = "image_data/piece0.jpg"
# im_pth = "image_data/test.jpg"

image = cv2.imread(im_pth,cv2.IMREAD_UNCHANGED)#,cv2.IMREAD_GRAYSCALE)
# template=cv2.imread('../client_yaml/temp1.jpg')#,cv2.IMREAD_GRAYSCALE)




# First create the image with alpha channel
rgba = cv2.cvtColor(image, cv2.COLOR_RGB2RGBA)

# Then assign the mask to the last channel of the image
# rgba[:, :, -1] = 0


for r in range(len(rgba)):
	for c in range(len(rgba[0])):
		if rgba[r][c][0]==0 and rgba[r][c][1]==0 and rgba[r][c][2]==0:
			rgba[r][c][-1]=0
print(rgba[:,:,3])

cv2.imshow("image", rgba)
cv2.waitKey(0)
cv2.destroyAllWindows()


cv2.imwrite('image_data/dst.png', rgba, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])