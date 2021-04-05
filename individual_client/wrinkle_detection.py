import cv2, copy
import numpy as np


image=cv2.imread("image_data/depth7.jpg")        #read in image
roi=image[270:534,420:887,:]
roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

cv2.imshow('Gray image', roi)
cv2.waitKey(0)
cv2.destroyAllWindows()