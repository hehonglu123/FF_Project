import cv2, sys, yaml
from matplotlib import pyplot as plt
import numpy as np
sys.path.append('../toolbox/')
from temp_match import *
from plate_calib import *
from fabric_detection import detection


image=cv2.imread('image_data/military_pcb.jpg',0)
ROI, ppu=preprocess(image)
ROI[1][0]=400

#load template
template=cv2.flip(cv2.imread('../client_yaml/FR-LF-UP.jpg', cv2.IMREAD_GRAYSCALE),0)
with open('../client_yaml/fabric.yaml') as file:
	dimension = np.array(yaml.load(file)['FR-LF-UP'],dtype=np.float64)


template_ppu=len(template)*len(template[0])/(dimension[0]*dimension[1])

scale=ppu/template_ppu
template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

#convert to binary
template_binary= cv2.threshold(template, 50, 255, cv2.THRESH_BINARY)[1]

test_region=image[ROI[0][0]:ROI[0][1],ROI[1][0]:ROI[1][1]]
angle,center2=match_w_ori(test_region,template_binary,[0],'edge')
center2=np.flip(center2+np.flip(ROI[:,0]))


print(center2,angle)
# ###show final results
cv2.circle(image, (int(center2[1]),int(center2[0])),10,(0,0,255), -1)		
cv2.imshow("image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()


