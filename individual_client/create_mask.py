import numpy as np
import cv2, time, sys, yaml
sys.path.append('../toolbox/')
from vision import *

fabric_name='PD19_016C-FR-LFT-UP HICKEY V2 36'
image=cv2.imread('../vision_check.jpg',0)

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

# get the (largest) contour
contours = cv2.findContours(template, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]
big_contour = max(contours, key=cv2.contourArea)

# draw white filled contour on black background
result = np.zeros_like(template)
cv2.drawContours(result, [big_contour], 0, (255,255,255), cv2.FILLED)


cv2.imshow('result', result)
cv2.waitKey(0)
cv2.destroyAllWindows()