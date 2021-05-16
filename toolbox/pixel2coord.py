import numpy as np
import yaml, traceback
import sympy as sym



def pixel2coord(R,p,pixel,z):
	with open(r'/home/rpi/FF_Project/calibration/camera_extrinsic.yaml') as file:
		dict_file = yaml.load(file, Loader=yaml.FullLoader)

	R=sym.Matrix(dict_file['R'])
	p=sym.Matrix(dict_file['p'])

	with open(r'/home/rpi/FF_Project/calibration/camera_intrinsic.yaml') as file:
		dict_file = yaml.load(file, Loader=yaml.FullLoader)

	mtx=sym.Matrix(dict_file['mtx'])


	x = sym.Symbol('x')
	y = sym.Symbol('y')
	s = sym.Symbol('s')
	world_coord=sym.Matrix([[x],[y],[z]])


	cam_coord=R*world_coord

	temp_right=mtx*(cam_coord+p)
	temp_left=s*sym.Matrix([[pixel[0]],[pixel[1]],[1]])
	temp=temp_left-temp_right

	out=list(sym.linsolve([temp[0],temp[1],temp[2]], (x,y,s)))[0]

	return np.array([out[0],out[1],z])


def pixel2coord2(R,p,pixel,z):
	with open(r'/home/rpi/FF_Project/calibration/camera_extrinsic.yaml') as file:
		dict_file = yaml.load(file, Loader=yaml.FullLoader)

	R=dict_file['R']
	p=dict_file['p']
	with open(r'/home/rpi/FF_Project/calibration/camera_intrinsic.yaml') as file:
		dict_file = yaml.load(file, Loader=yaml.FullLoader)

	mtx=dict_file['mtx']
    dist = dict_file['dist']

    #TODO: Figure out a better value for this
    object_z_cam_dist = p[-1]-z

    # Find the corresponding world pose of the detected pose in camera frame
    dst = cv2.undistortPoints(pixel,mtx,dist) # dst is Xc/Zc and Yc/Zc in the same shape of src
    dst = dst * float(object_z_cam_dist) * 1000.0 # Multiply by given Zc distance to find all cordinates, multiply by 1000 is because of Zc is given in meters but others are in millimeters
    dst = np.squeeze(dst) * 0.001 # Xc and Yc as vector

    # Finally the translation between the detected object center and the camera frame represented in camera frame is T = [Xc,Yc,Zc]
    Xc = dst[0]
    Yc = dst[1]
    Zc = float(z)
    T = np.asarray([Xc,Yc,Zc])

    return np.dot(R,T)+p