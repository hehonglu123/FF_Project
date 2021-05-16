import time
import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(10,7,.0254,.02,aruco_dict)
img = board.draw((2794,2159))
cv2.imwrite('charuco.png',img)
