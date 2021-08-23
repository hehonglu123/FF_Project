import cv2, sys, yaml
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from temp_match import *
from fabric_detection import detection

def read_template(im_path,dimension,ppu):
	#load template
	template=cv2.imread(im_path, cv2.IMREAD_GRAYSCALE)
	
	template_ppu=len(template)*len(template[0])/(dimension[0]*dimension[1])

	scale=ppu/template_ppu

	template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

	#convert to binary
	template_binary= cv2.threshold(template, 40, 255, cv2.THRESH_BINARY)[1]

	return template_binary


image=cv2.imread('image_data/interlining_pcb.jpg',0)
edge_raw=cv2.imread('../client_yaml/plate_edge.jpg',0)


with open('../client_yaml/vision.yaml') as file:
	vision_yaml = yaml.load(file, Loader=yaml.FullLoader)
with open('../client_yaml/fabric.yaml') as file:
	fabric_dimension = yaml.load(file, Loader=yaml.FullLoader)

ROI=vision_yaml['ROI']
ppu=vision_yaml['ppu']

#load template
template=read_template('../client_yaml/FR-LF-UP.jpg',fabric_dimension['FR-LF-UP'],ppu)
template_ppu=len(template)*len(template[0])/(fabric_dimension['FR-LF-UP'][0]*fabric_dimension['FR-LF-UP'][1])

scale=ppu/template_ppu

template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

#convert to binary
template_binary= cv2.threshold(template, 40, 255, cv2.THRESH_BINARY)[1]

roi_frame=image[ROI[0]:ROI[1],ROI[2]:ROI[3]]
angle,center=match_w_ori(roi_frame,template_binary,0.,'edge',edge_raw)


print(center,angle)
print(np.array([len(roi_frame[0]),len(roi_frame)])/2)
offset_p=(center-np.array([len(roi_frame[0]),len(roi_frame)])/2.)/ppu
print(offset_p)
# ###show final results
cv2.circle(roi_frame, (int(center[0]),int(center[1])),10,(255,255,255), -1)		
cv2.imshow("image", roi_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()


