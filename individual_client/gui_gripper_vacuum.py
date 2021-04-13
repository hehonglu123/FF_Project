#!/usr/bin/python3
from RobotRaconteur.Client import *
import RobotRaconteur as RR
import sys, os, time, argparse, traceback
from tkinter import *
from tkinter import messagebox
import numpy as np
  
#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--tool-name",type=str)
args, _ = parser.parse_known_args()
tool_name=args.tool_name


#auto discovery
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.tool.Tool",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if tool_name in serviceinfo2.NodeName:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()



#connect
tool_sub=RRN.SubscribeService(url)
tool=tool_sub.GetDefaultClientWait(1)
state_w = tool_sub.SubscribeWire("tool_state")



top=Tk()
top.title(tool_name)
jobid = None
def gripper_ctrl1(tool):

	if gripper1.config('relief')[-1] == 'sunken':
		tool.open()
		gripper1.config(relief="raised")
		gripper1.configure(bg='red')
		gripper1.configure(text='gripper off')

	else:
		tool.close()
		gripper1.config(relief="sunken")
		gripper1.configure(bg='green')
		gripper1.configure(text='gripper on')
	return



label = Label(top, fg = "black", justify=LEFT)
label.pack()



gripper1=Button(top,text='gripper1 off',command=lambda: gripper_ctrl1(tool),bg='red')


gripper1.pack()

top.mainloop()
