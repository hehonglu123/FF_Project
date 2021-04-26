from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback


url='rr+tcp://192.168.50.107:23232/?service=stretch'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
lift=robot.get_lift()
arm=robot.get_arm()
end_of_arm=robot.get_end_of_arm()
base=robot.get_base()
arm_status=arm.status_rr.Connect()
lift_status=lift.status_rr.Connect()
base_status=base.status_rr.Connect()

# time.sleep(2)
# print(lift_status.InValue['pos'])
for i in range(5):
	print(i)
	lift.move_to(0.85)
	robot.push_command()
	time.sleep(4)
	lift.move_to(1.09)
	robot.push_command()
	time.sleep(4)
