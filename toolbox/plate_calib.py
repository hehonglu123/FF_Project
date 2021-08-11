import cv2, sys
from matplotlib import pyplot as plt
import numpy as np
from cv2 import aruco


def aruco_process(frame):
	aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
	parameters =  aruco.DetectorParameters_create()
	parameters.adaptiveThreshWinSizeMin=5
	corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
	tag_centroids=np.average(corners,axis=2)
	return tag_centroids, ids

def PolyArea2D(pts):
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area
###determine pixels/unit distance, ROI
def preprocess(image):
	tag_centroids, ids = aruco_process(image)
	ROI=np.array([[np.min(tag_centroids[:,:,1]),np.max(tag_centroids[:,:,1])],[np.min(tag_centroids[:,:,0]),np.max(tag_centroids[:,:,0])]]).astype(int)		#[[r1,r2],[c1,c2]]
	ppu=PolyArea2D(np.squeeze(tag_centroids,axis=1))/358800		#pixel area / plate area in mm
	return ROI,ppu

