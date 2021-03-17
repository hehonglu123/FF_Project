import cv2
import numpy as np
def detection(image,palette,tolerance=np.array([20,20,20])):
	image_size=len(image)*len(image[0]) #get image size
	image_dimension=np.array([len(image),len(image[0])])    #get image dimension

	for color in palette:

		filtered=cv2.inRange(image,color-tolerance,color+tolerance) #filter the image with upper bound and lower bound in bgr format
		#run color connected components to filter the counts and centroid
		retval, labels, stats, centroids=cv2.connectedComponentsWithStats(filtered) #run CCC on the filtered image
		idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.99*image_size))[0]   #threshold the components to find the best one
		for i in idx:
			pixels=np.where(labels == i)
			if filtered[pixels[0][0],pixels[1][0]]==255:
				#show filtered image
				temp=np.where(labels != i, 0, labels)
				temp=np.where(temp == i,1, temp)
				temp=np.expand_dims(temp, axis=-1)
				temp=temp*np.array([color])
				cv2.namedWindow("Image")
				cv2.imshow("Image",temp.astype(np.uint8))
				cv2.waitKey()
	# return labels[idx]

image=cv2.imread("images/fabric_1.jpg")        #read in image
# palette=np.array([[46,50,229],[245,245,245]])
palette=np.array([[200,90,50],[160,200,190],[130,140,250]])
detection(image,palette,tolerance=np.array([30,30,30]))