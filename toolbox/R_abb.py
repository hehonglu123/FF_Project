import numpy as np
def R_ee_down(angle):		
	R=np.array([[0, -np.sin(angle),np.cos(angle)],
			[0, np.cos(angle), np.sin(angle)],
			[-1,0,0]])
	return R
def R_ee_up(angle):			#need modify
	R=np.array([[0, np.sin(angle),np.cos(angle)],
			[0, -np.cos(angle), np.sin(angle)],
			[1,0,0]])
	return R
def R_ee_tilt(angle):			#need modify
	R=np.array([[0, 0, 1],
			[np.sin(angle), np.cos(angle), 0],
			[-np.cos(angle),np.sin(angle),0]])
	return R