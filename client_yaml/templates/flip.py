import cv2


fabric_names=['PD19_016C-FR-RGT-LWR HICKEY 36',
'PD19_016C-FR-RGT-LWR HICKEY 44',
'PD19_016C-FR-RGT-LWR HICKEY 56',
'PD19_016C-FR-RGT-LWR-INT HICKEY 36',
'PD19_016C-FR-RGT-LWR-INT HICKEY 44',
'PD19_016C-FR-RGT-LWR-INT HICKEY 56',
'PD19_016C-FR-RGT-UP HICKEY 36',
'PD19_016C-FR-RGT-UP HICKEY 44',
'PD19_016C-FR-RGT-UP HICKEY 56',
'PD19_016C-FR-RGT-UP-INT HICKEY 36',
'PD19_016C-FR-RGT-UP-INT HICKEY 44',
'PD19_016C-FR-RGT-UP-INT HICKEY 56']

for fabric_name in fabric_names:
	temp=cv2.imread(fabric_name+'.jpg')
	cv2.imwrite(fabric_name+'.jpg',cv2.flip(temp, 0))

