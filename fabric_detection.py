import cv2
import numpy as np
def detection(image,palette,tolerance=np.array([20,20,50])):
	image_size=len(image)*len(image[0]) #get image size
	image_dimension=np.array([len(image),len(image[0])])    #get image dimension

	for color in palette:
		hsv_color=cv2.cvtColor(np.array([[color]],dtype=np.uint8), cv2.COLOR_BGR2HSV)
		hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		filtered=cv2.inRange(hsv_image,hsv_color-tolerance,hsv_color+tolerance) #filter the image with upper bound and lower bound in bgr format

		#run color connected components to filter the counts and centroid
		retval, labels, stats, centroids=cv2.connectedComponentsWithStats(filtered) #run CCC on the filtered image
		idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.99*image_size))[0]   #threshold the components to find the best one
		for i in idx:
			pixels=np.where(labels == i)
			if filtered[pixels[0][0],pixels[1][0]]==255:
				#show filtered image
				temp=np.where(labels != i, 0, labels)
				temp=np.where(temp == i,1, temp)
				contours=cv2.findContours(temp.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
				temp=np.expand_dims(temp, axis=-1)
				temp=temp*np.array([color])
				
				hull_list = []
				for contour in contours:
					#filter out small noise area
					if cv2.contourArea(contour)>0.01*image_size:
						hull = cv2.convexHull(contour)
						hull_list.append(hull)
				# Draw contours + hull results
				for i in range(len(hull_list)):
					color = ( int (color [ 0 ]), int (color [ 1 ]), int (color [ 2 ])) 
					cv2.drawContours(temp, hull_list, i, color)
				# Show in a window
				cv2.namedWindow("Image")
				cv2.imshow("Image",temp.astype(np.uint8))
				cv2.waitKey()
	# return labels[idx]

image=cv2.imread("images/fabric_1.jpg")        #read in image
#palette in BGR
# palette=np.array([[46,50,229],[245,245,245]])
palette=np.array([[200,90,50],[160,200,190],[130,140,250]])
detection(image,palette,tolerance=np.array([30,30,100]))