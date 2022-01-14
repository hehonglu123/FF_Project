import numpy as np
import cv2, time
def read_template(im_path,dimension,ppu):
	#load template
	template=cv2.imread(im_path, cv2.IMREAD_GRAYSCALE)
	
	template_ppu=len(template)*len(template[0])/(dimension[0]*dimension[1])

	scale=ppu/template_ppu

	template=cv2.resize(template,tuple((np.sqrt(scale)*np.flip(np.array(template.shape[:2]))).astype(int)))

	#convert to binary
	template_binary= cv2.threshold(template, 40, 255, cv2.THRESH_BINARY)[1]

	return template_binary

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



def edge_temp_match(image,template,interlining=False):	#edge based match with alpha channel
	# cv2.imshow("image", image)
	# cv2.imshow("template", template)
	# cv2.waitKey(0)

	#matching
	try:
		if interlining:
			res = cv2.matchTemplate(image, template , cv2.TM_SQDIFF, mask=template)
		else:
			res = cv2.matchTemplate(image, template , cv2.TM_SQDIFF, mask=create_mask(template))
	except cv2.error:			#when plate not able to cover all fabric
		return 99999999999, (0,0)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc

def bold_edge(image,num_pix=10):
	bolded = cv2.filter2D(image,-1,np.ones((num_pix,num_pix))/num_pix**2,borderType=cv2.BORDER_CONSTANT)
	bolded = cv2.threshold(bolded, 5, 255, cv2.THRESH_BINARY)[1]
	return bolded

def create_mask(template):
	# get the (largest) contour
	contours = cv2.findContours(template, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours = contours[0] if len(contours) == 2 else contours[1]
	big_contour = max(contours, key=cv2.contourArea)

	# draw white filled contour on black background
	result = np.zeros_like(template)
	cv2.drawContours(result, [big_contour], 0, (255,255,255), cv2.FILLED)
	return result

def match_w_ori(image,template,orientation,alg='hsva',edge_raw=None,angle_range=10,angle_resolution=1.,interlining=False):
	min_error=9999999999
	act_angle=0
	loc=[0,0]

	orientation=np.degrees(orientation)


	tEdged = template#cv2.Canny(template, 50, 200,apertureSize =3)
	image_edge=cv2.Canny(image, 50, 200,apertureSize =3)
	
	edged = bold_edge(image_edge)
	try:
		edged=cv2.subtract(edged,edge_raw)
		# cv2.imshow("image", edged)
		# cv2.waitKey(0)
	except:
		# traceback.print_exc()
		pass

	# cv2.imwrite('edge_raw.jpg',edged)
	# cv2.imshow("image", edged)
	# cv2.imshow("template", tEdged)
	# cv2.waitKey(0)

	# for i in range(0,181,180):
	for angle in np.arange(orientation-angle_range,orientation+angle_range,angle_resolution):
	
		template_rt=rotate_image(tEdged,angle,[0,0,0])
		###make template binary
		template_rt=cv2.threshold(template_rt, 50, 255, cv2.THRESH_BINARY)[1]

		min_val, min_loc=edge_temp_match(edged,template_rt,interlining)


		if min_val<min_error:
			min_error=min_val
			act_angle=angle
			loc=min_loc
			w=len(template_rt[0])
			h=len(template_rt)
			loc=(min_loc[0]+w/2,min_loc[1]+h/2)



	return act_angle,loc,min_error


def match_w_ori_single(image,template,orientation,alg='hsva',edge_raw=None,interlining=False):

	min_error=9999999999
	act_angle=0

	orientation=np.degrees(orientation)


	tEdged = template#cv2.Canny(template, 50, 200,apertureSize =3)
	
	image_edge=cv2.Canny(image, 50, 200,apertureSize =3)
	try:
		image_edge_tmp=cv2.subtract(image_edge,edge_raw)

	except:
		# traceback.print_exc()
		pass
	edged = bold_edge(image_edge)


	
	# for i in range(0,181,180):
	angle=orientation

	template_rt=rotate_image(tEdged,angle,[0,0,0])
	###make template binary
	template_rt=cv2.threshold(template_rt, 50, 255, cv2.THRESH_BINARY)[1]
	min_val, min_loc=edge_temp_match(edged,template_rt,interlining)


	w=len(template_rt[0])
	h=len(template_rt)

	return angle,(min_loc[0]+w/2,min_loc[1]+h/2)