import cv2, copy
import numpy as np
def filter(image,color,tolerance):
	hsv_color=cv2.cvtColor(np.array([[color]],dtype=np.uint8), cv2.COLOR_BGR2HSV)
	hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	filtered=cv2.inRange(hsv_image,hsv_color-tolerance,hsv_color+tolerance) #filter the image with upper bound and lower bound in bgr format
	return filtered
def get_label_image(labels,idx):
	image=np.where(labels != idx, 0, labels)
	image=np.where(image == idx,1, image)
	return image

#calculate orientation of binary image
def get_orientation(bn_image):
	idx_tp=np.nonzero(bn_image)
	idx=np.vstack(idx_tp)
	cov=np.cov(idx)
	w,v=np.linalg.eig(cov)
	eigv=v[np.argmax(w)]	##(r,c)=>(y,x)
	orientation=np.arctan2(eigv[0],eigv[1])
	return orientation

def bw2cl(image,color):
	return np.expand_dims(image, axis=-1)*np.array(color)

def detection(image,color,tolerance=np.array([20,20,50])):
	image_size=len(image)*len(image[0]) #get image size
	image_dimension=np.array([len(image),len(image[0])])    #get image dimension
	final=np.zeros((len(image),len(image[0]),3), np.uint8)

	#adaptive palette
	filtered=filter(image,color,tolerance)

	#run color connected components to filter the counts and centroid
	retval, labels, stats, centroids=cv2.connectedComponentsWithStats(filtered) #run CCC on the filtered image
	idx=np.where(np.logical_and(stats[:,4]>=0.005*image_size, stats[:,4]<=0.99*image_size))[0]   #threshold the components to find the best one
	
	for i in idx:
		pixels=np.where(labels == i)
		if filtered[pixels[0][0],pixels[1][0]]==255:

			#show filtered image
			bn_image=get_label_image(labels,i)
			temp=bw2cl(bn_image,[255,255,255])

			num=stats[i,4]
			result = cv2.bitwise_and(image, temp.astype(np.uint8))
			result_sum=np.sum(np.sum(result,axis=0),axis=0)
			color=result_sum/num
			break
	#use average color to filter
	filtered=filter(image,color,tolerance)

	#run color connected components to filter the counts and centroid
	retval, labels, stats, centroids=cv2.connectedComponentsWithStats(filtered) #run CCC on the filtered image
	idx=np.where(np.logical_and(stats[:,4]>=0.005*image_size, stats[:,4]<=0.99*image_size))[0]   #threshold the components to find the best one

	orientation=[]
	centroid=[]
	for i in idx:
		pixels=np.where(labels == i)
		if filtered[pixels[0][0],pixels[1][0]]==255:

			#show filtered image
			bn_image=get_label_image(labels,i)
			contours=cv2.findContours(bn_image.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
			temp=bw2cl(bn_image,color)

			orientation.append(get_orientation(bn_image))
			centroid.append(np.flip(centroids[i]))
			
			hull_list = []
			for contour in contours:
				#filter out small noise area
				if cv2.contourArea(contour)>0.01*image_size:
					hull = cv2.convexHull(contour)
					hull_list.append(hull)
			# Draw contours + hull results
			color = ( int (color [ 0 ]), int (color [ 1 ]), int (color [ 2 ])) 
			cv2.drawContours(temp, hull_list, -1, color)
			# cv2.imwrite('images/6_'+str(i)+'.jpg',temp.astype(np.uint8))
			final+=temp.astype(np.uint8)

	return (orientation,centroid)

image=cv2.imread("individual_client/image_data/rgb3.jpg")        #read in image
#palette in BGR

# palette=np.array([[29,27,19],[57,37,20]])
color=np.array([29,27,19])

(orientation,centroid)=detection(image,color,tolerance=np.array([30,30,100]))
print(orientation,centroid)