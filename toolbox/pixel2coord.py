import numpy as np
import yaml, traceback
import sympy as sym



def convert(R,p,pixel,z):
	with open(r'../calibration/camera_extrinsic.yaml') as file:
		dict_file = yaml.load(file, Loader=yaml.FullLoader)

	R=sym.Matrix(dict_file['R'])
	p=sym.Matrix(dict_file['p'])

	with open(r'../calibration/camera_intrinsic.yaml') as file:
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

