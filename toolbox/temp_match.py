import numpy as np
import cv2
BACKGROUND=[170,166,161]


def rotate_image(mat, angle, background):
	"""
	Rotates an image (angle in degrees) and expands image to avoid cropping
	"""

	height, width = mat.shape[:2] # image shape has 3 dimensions

	image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape

	rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

	# rotation calculates the cos and sin, taking absolutes of those.
	abs_cos = abs(rotation_mat[0,0]) 
	abs_sin = abs(rotation_mat[0,1])

	# find the new width and height bounds
	bound_w = int(height * abs_sin + width * abs_cos)
	bound_h = int(height * abs_cos + width * abs_sin)

	# subtract old image center (bringing image back to origo) and adding the new image center coordinates
	rotation_mat[0, 2] += bound_w/2 - image_center[0]
	rotation_mat[1, 2] += bound_h/2 - image_center[1]

	# rotate image with the new bounds and translated rotation matrix
	rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h),cv2.BORDER_CONSTANT,borderValue=background)
	return rotated_mat
def square(im,desired_size, background):
	old_size = im.shape[:2] # old_size is in (height, width) format

	ratio = float(desired_size)/max(old_size)
	new_size = tuple([int(x*ratio) for x in old_size])

	# new_size should be in (width, height) format

	im = cv2.resize(im, (new_size[1], new_size[0]))

	delta_w = desired_size - new_size[1]
	delta_h = desired_size - new_size[0]
	top, bottom = delta_h//2, delta_h-(delta_h//2)
	left, right = delta_w//2, delta_w-(delta_w//2)

	new_im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT,value=background)
	return new_im

def color_temp_match(image,template):	#rgb based match with background
	template=cv2.cvtColor(template,cv2.COLOR_BGRA2BGR)
	template[np.where((template==[0,0,0]).all(axis=2))] = BACKGROUND
	template[np.where((template==[255,255,255]).all(axis=2))] = BACKGROUND
	# cv2.imshow("image", image)
	# cv2.imshow("template", template)
	# cv2.waitKey(0)
	#matching
	res = cv2.matchTemplate(image, template, cv2.TM_SQDIFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc

def hsv_temp_match(image,template):		#hsv based match with background, 0.5 weight on value
	template=cv2.cvtColor(template,cv2.COLOR_BGRA2BGR)
	template[np.where((template==[0,0,0]).all(axis=2))] = BACKGROUND
	template[np.where((template==[255,255,255]).all(axis=2))] = BACKGROUND

	# cv2.imshow("template",template)
	# cv2.waitKey(0)

	template=cv2.cvtColor(template,cv2.COLOR_BGR2HSV)
	image=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
	mask=np.ones(template.shape,dtype=np.float32)
	mask[:,:,-1]=0.5
	#matching
	res = cv2.matchTemplate(image, template, cv2.TM_SQDIFF, mask=mask)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc

def hsv_temp_match2(image,template):		#color based match with alpha channel
	channels = cv2.split(template)
	mask = np.moveaxis(np.tile(np.array(channels[3]),(3,1,1)),0,-1)
	template=cv2.cvtColor(template,cv2.COLOR_BGRA2BGR)


	# cv2.imshow("image", image)
	# cv2.imshow("template",template)
	# cv2.imshow("mask", mask)
	# cv2.waitKey(0)

	image=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
	template=cv2.cvtColor(template,cv2.COLOR_BGR2HSV)
	mask[:,:,-1]=0.5		#0.5 weight on value
	#matching
	res = cv2.matchTemplate(image, template, cv2.TM_SQDIFF,mask=mask)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc

def color_temp_match2(image,template):		#color based match with alpha channel
	channels = cv2.split(template)
	mask = np.moveaxis(np.tile(np.array(channels[3]),(3,1,1)),0,-1)
	template=cv2.cvtColor(template,cv2.COLOR_BGRA2BGR)

	# cv2.imshow("image", image)
	# cv2.imshow("template",template)
	# cv2.imshow("mask", mask)
	# cv2.waitKey(0)
	#matching
	res = cv2.matchTemplate(image, template, cv2.TM_SQDIFF,mask=mask)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc



def bw_temp_match(image,template):		#binary based match with background
	image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	template=cv2.cvtColor(template,cv2.COLOR_BGRA2GRAY)
	template[np.where(template==0)]=sum(BACKGROUND)/len(BACKGROUND)
	template[np.where(template==255)]=sum(BACKGROUND)/len(BACKGROUND)
	# cv2.imshow("image", image)
	# cv2.imshow("template", template)
	# cv2.waitKey(0)
	#matching
	res = cv2.matchTemplate(image, template, cv2.TM_SQDIFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc

def bw_temp_match2(image,template):		#binary based match with alpha channel
	#convert to bw
	image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	

	#create mask
	channels = cv2.split(template)
	mask = np.moveaxis(np.tile(np.array(channels[-1]),(1,1,1)),0,-1)
	#convert to bw
	template=cv2.cvtColor(template,cv2.COLOR_BGRA2GRAY)

	# cv2.imshow("image", image)
	# cv2.imshow("template", template)
	# cv2.imshow("mask", mask)
	# cv2.waitKey(0)
	#matching
	res = cv2.matchTemplate(image, template, cv2.TM_SQDIFF, mask=mask)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc


def edge_temp_match(image,template):	#edge based match with alpha channel
	# cv2.imshow("image", template)
	# cv2.imshow("template", image)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	#matching
	res = cv2.matchTemplate(image, template , cv2.TM_SQDIFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc

temp_alg={'edge':edge_temp_match,'rgb':color_temp_match,'hsv':hsv_temp_match,'rgba':color_temp_match2,'hsva':hsv_temp_match2,'bw':bw_temp_match,'bwa':bw_temp_match2}


def match(image,template,alg='hsva'):
	min_error=9999999999
	act_angle=0
	
	


	for angle in range(0,360,2):
		template_rt=rotate_image(template,angle,[0,0,0,0])
		min_val, min_loc=temp_alg['hsva'](image,template_rt)


		if min_val<min_error:
			min_error=min_val
			act_angle=angle
			loc=min_loc
			w=len(template_rt[0])
			h=len(template_rt)
			loc=(min_loc[0]+w/2,min_loc[1]+h/2)


		# print(angle,min_loc,min_val)
	return act_angle,loc

def match_w_ori(image,template,orientation,alg='hsva'):
	min_error=9999999999
	act_angle=0

	orientation=round(np.degrees(orientation)[0])

	tEdged = cv2.Canny(template, 50, 200)
	edged = cv2.Canny(image, 50, 200)
	for i in range(0,181,180):
		for angle in range(orientation+i-5,orientation+i+5):
			template_rt=rotate_image(template,angle,[0,0,0,0])
			min_val, min_loc=temp_alg[alg](image,template_rt)
			# template_rt=rotate_image(tEdged,angle,[0,0,0])
			# min_val, min_loc=temp_alg['edge'](edged,template_rt)
			

			if min_val<min_error:
				min_error=min_val
				act_angle=angle
				loc=min_loc
				w=len(template_rt[0])
				h=len(template_rt)
				loc=(min_loc[0]+w/2,min_loc[1]+h/2)

	template=cv2.cvtColor(template,cv2.COLOR_BGRA2BGR)
	template[np.where((template==[0,0,0]).all(axis=2))] = BACKGROUND
	template_rt=rotate_image(template,act_angle,BACKGROUND)



	return act_angle,loc