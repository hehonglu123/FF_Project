import numpy as np
def R_ee(angle):		
	R=np.array([[0, -np.sin(angle),np.cos(angle)],
			[0, np.cos(angle), np.sin(angle)],
			[-1,0,0]])
	return R
def R_ee_up(angle):			#need modify
	R=np.array([[0, np.sin(angle),-np.cos(angle)],
			[0, np.cos(angle), np.sin(angle)],
			[1,0,0]])
	return R
def R_ee_tilt_x(angle):			#need modify
	R=np.array([[0, 0, 1],
			[np.sin(angle), np.cos(angle), 0],
			[-np.cos(angle),np.sin(angle),0]])
	return R

def R_ee_tilt_y(angle):			#need modify
	R=np.array([[np.sin(angle), np.cos(angle), 0],
			[0,0,1],
			[-np.cos(angle),np.sin(angle),0]])
	return R