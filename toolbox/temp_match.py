import numpy as np
import cv2
BACKGROUND=[0,0,0]

def rotate_image(mat, angle):
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
	rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h),cv2.BORDER_CONSTANT,borderValue=BACKGROUND)
	return rotated_mat
def square(im,desired_size):
	old_size = im.shape[:2] # old_size is in (height, width) format

	ratio = float(desired_size)/max(old_size)
	new_size = tuple([int(x*ratio) for x in old_size])

	# new_size should be in (width, height) format

	im = cv2.resize(im, (new_size[1], new_size[0]))

	delta_w = desired_size - new_size[1]
	delta_h = desired_size - new_size[0]
	top, bottom = delta_h//2, delta_h-(delta_h//2)
	left, right = delta_w//2, delta_w-(delta_w//2)

	new_im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT,value=BACKGROUND)
	return new_im

def color_temp_match(image,template):
	diff=np.zeros((len(image)-len(template),len(image[0])-len(template[0])))
	for r in range(len(image)-len(template)):
		for c in range(len(image[0])-len(template[0])):
			img_diff=image[r:r+len(template),c:c+len(template[0])]-template
				
			diff[r][c]=np.linalg.norm(img_diff)

	return diff.min(), np.flip(np.unravel_index(diff.argmin(), diff.shape))

def edge_temp_match(image,template):
	
	cv2.imshow("image", image)
	cv2.imshow("template", template)
	cv2.waitKey(0)
	# cv2.destroyAllWindows()

	#matching
	res = cv2.matchTemplate(template, image, cv2.TM_SQDIFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

	return min_val, min_loc

def match(image,template):
	min_error=9999999999
	act_angle=0
	
	tEdged = cv2.Canny(template, 50, 200)
	edged = cv2.Canny(image, 50, 200)

	for angle in range(0,360,5):
			
		template_rt=rotate_image(tEdged,angle)
		template_rt=square(template_rt,np.max(np.shape(template_rt)))
		min_val, min_loc=edge_temp_match(edged,template_rt)
		
		# template_rt=rotate_image(template,angle)
		# template_rt=square(template_rt,np.max(np.shape(template_rt)))
		# min_val, min_loc=color_temp_match(image,template)
		

		if min_val<min_error:
			min_error=min_val
			act_angle=angle
			loc=min_loc
			w=len(template_rt[0])
			h=len(template_rt)
			loc=(min_loc[0]+w/2,min_loc[1]+h/2)


		print(angle,min_loc,min_val)
	return act_angle,loc

