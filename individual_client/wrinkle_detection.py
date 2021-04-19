import cv2, copy
import numpy as np


depth_data=np.load("image_data/depth0.npy")        #read in image
print(depth_data)
# roi=image[270:534,420:887,:]

# cv2.imshow('Gray image', roi)
# cv2.waitKey(0)
# cv2.destroyAllWindows()