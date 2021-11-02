import cv2, sys, yaml
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from vision import *
import time

fabric_name='PD19_016C-FR-LFT-UP HICKEY V2 36'
image=cv2.imread('../vision_check.jpg',0)
edge_raw=cv2.imread('../edge_raw.jpg',0)


with open('../client_yaml/vision.yaml') as file:
	vision_yaml = yaml.load(file, Loader=yaml.FullLoader)
with open('../client_yaml/fabric.yaml') as file:
	fabric_dimension = yaml.load(file, Loader=yaml.FullLoader)

ROI=vision_yaml['ROI']
ppu=vision_yaml['ppu']

#load template
template=template=read_template('../client_yaml/templates/'+fabric_name+'.jpg',fabric_dimension[fabric_name],ppu)
template_ppu=len(template)*len(template[0])/(fabric_dimension[fabric_name][0]*fabric_dimension[fabric_name][1])

scale=ppu/template_ppu

template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

#convert to binary
template_binary= cv2.threshold(template, 40, 255, cv2.THRESH_BINARY)[1]

roi_frame=image[ROI[0]:ROI[1],ROI[2]:ROI[3]]

scale_percent = 1.
roi_frame_small = cv2.resize(roi_frame, (int(roi_frame.shape[1] * scale_percent),int(roi_frame.shape[0] * scale_percent)))#, interpolation = cv2.INTER_AREA)
template_binary_small = cv2.resize(template_binary, (int(template_binary.shape[1] * scale_percent),int(template_binary.shape[0] * scale_percent)))

angle,center=match_w_ori(roi_frame_small,template_binary_small,0.,'edge',edge_raw=None)


center_original=[center[0]/scale_percent,center[1]/scale_percent]

print(center,angle)
print(np.array([len(roi_frame[0]),len(roi_frame)])/2)
offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)/ppu
print(offset_p)
# ###show final results
cv2.circle(roi_frame, (int(center_original[0]),int(center_original[1])),10,(255,255,255), -1)		
cv2.imshow("image", roi_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()


