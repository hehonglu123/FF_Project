import time
import numpy as np
def jog_joint(robot,vel_ctrl,q,t):
	#parameter setup

	n= len(robot.robot_info.joint_info)

	#enable velocity mode
	vel_ctrl.enable_velocity_mode()
	
	qdot=1.15*(q-vel_ctrl.joint_position())/t
	now=time.time()
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.01 and time.time()-now<t:
		vel_ctrl.set_velocity_command(qdot)


	vel_ctrl.set_velocity_command(np.zeros((n,)))
	vel_ctrl.disable_velocity_mode() 
