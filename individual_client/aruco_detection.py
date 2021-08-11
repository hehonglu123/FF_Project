import cv2, sys
from matplotlib import pyplot as plt
import numpy as np
from cv2 import aruco


def aruco_process(frame):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
	parameters =  aruco.DetectorParameters_create()
	# parameters.minMarkerPerimeterRate=0.00001
	# parameters.adaptiveThreshConstant=20
	# # parameters.minMarkerDistanceRate=0.005
	parameters.adaptiveThreshWinSizeMin=5
	# parameters.adaptiveThreshWinSizeMax=10
	# parameters.adaptiveThreshWinSizeStep=1

	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
	return corners, ids, frame_markers

im_pth = "image_data/military_pcb.jpg"

image = cv2.imread(im_pth,cv2.IMREAD_UNCHANGED)#,cv2.IMREAD_GRAYSCALE)

corners, ids, frame_markers=aruco_process(image)
tag_centroids=np.average(corners,axis=2)
print(tag_centroids,ids	)


cv2.imshow("image", frame_markers)
cv2.waitKey(0)
cv2.destroyAllWindows()