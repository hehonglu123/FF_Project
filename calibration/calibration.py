from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, cv2, copy, yaml
import numpy as np
sys.path.append('toolbox/')
from abb_ik import *

robot_sub=RRN.SubscribeService('rr+tcp://pi_fuse:58651?service=robot')
robot=robot_sub.GetDefaultClientWait(1)
state_w = robot_sub.SubscribeWire("robot_state")

testbed_yaml={}
print('robot connected')

testbed_yaml['home']=[0.5,0.,0.6]


print('move to bin1 position, press <Enter> if reached')

bin1_q=state_w.InValue.joint_position
bin1_pose=fwd(q)
testbed_yaml['bin1_p']=bin1_pose.p
testbed_yaml['bin1_R']=bin1_pose.R

print('saved')

print('move to bin2 position, press <Enter> if reached')

bin2_q=state_w.InValue.joint_position
bin2_pose=fwd(q)
testbed_yaml['bin2_p']=bin2_pose.p
testbed_yaml['bin2_R']=bin2_pose.R

print('saved')

print('move to placing position, press <Enter> if reached')
place_pose_q=state_w.InValue.joint_position
place_pose=fwd(q)
testbed_yaml['place_position']=place_pose.p

print('saved')


print('move to vision position, press <Enter> if reached')
testbed_yaml['vision_q']=vision_q

print('saved')

yaml.dump('../client_yaml/testbed.yaml')
