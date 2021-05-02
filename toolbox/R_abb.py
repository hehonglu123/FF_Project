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

def R_ee_tilt_y(angle,pitch=np.pi/3,yaw=0):			#need modify
	R=np.array([[np.sin(pitch), np.cos(pitch), 0],
			[0,0,1],
			[-np.cos(pitch),np.sin(pitch),0]])
	
	R_pitch=np.array([[np.cos(pitch), 0, np.sin(pitch)],
			[0,1,0],
			[-np.sin(pitch),0,np.cos(pitch)]])


	R_yaw=np.array([[np.cos(yaw), -np.sin(yaw),0],
			[np.sin(yaw), np.cos(yaw), 0],
			[0,0,1]])
	return np.dot(R_yaw,np.dot(R_pitch,R_ee(angle)))