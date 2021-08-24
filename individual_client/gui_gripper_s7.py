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
		tool.setf_param('grip1',RR.VarValue(False,'bool'))
		gripper1.config(relief="raised")
		gripper1.configure(bg='red')
		gripper1.configure(text='gripper off')

	else:
		tool.setf_param('grip1',RR.VarValue(True,'bool'))
		gripper1.config(relief="sunken")
		gripper1.configure(bg='green')
		gripper1.configure(text='gripper on')
	return
def roller_ctrl1(tool):

	if roller1.config('relief')[-1] == 'sunken':
		tool.setf_param('roll1',RR.VarValue(False,'bool'))
		roller1.config(relief="raised")
		roller1.configure(bg='red')
		roller1.configure(text='roller off')

	else:
		tool.setf_param('roll1',RR.VarValue(True,'bool'))
		roller1.config(relief="sunken")
		roller1.configure(bg='green')
		roller1.configure(text='roller on')
	return
def gripper_ctrl2(tool):

	if gripper2.config('relief')[-1] == 'sunken':
		tool.setf_param('grip2',RR.VarValue(False,'bool'))
		gripper2.config(relief="raised")
		gripper2.configure(bg='red')
		gripper2.configure(text='gripper off')

	else:
		tool.setf_param('grip2',RR.VarValue(True,'bool'))
		gripper2.config(relief="sunken")
		gripper2.configure(bg='green')
		gripper2.configure(text='gripper on')
	return
def roller_ctrl2(tool):

	if roller2.config('relief')[-1] == 'sunken':
		tool.setf_param('roll2',RR.VarValue(False,'bool'))
		roller2.config(relief="raised")
		roller2.configure(bg='red')
		roller2.configure(text='roller off')

	else:
		tool.setf_param('roll2',RR.VarValue(True,'bool'))
		roller2.config(relief="sunken")
		roller2.configure(bg='green')
		roller2.configure(text='roller on')
	return

##RR part
def update_label():
	tool_state=state_w.TryGetInValue()
	flags_text = "Tool State Flags:\n\n"
	if tool_state[0]:
		flags_text += 'sensor1: '
		if tool_state[1].sensor[0] != 0:
			flags_text += 'blocked' + "\n"
		else:
			flags_text += 'unblocked' + "\n"
		flags_text += 'switch1: '
		if tool_state[1].sensor[1] != 0:
			flags_text += 'pushed' + "\n"
		else:
			flags_text += 'released' + "\n"
		flags_text += 'sensor2: '
		if tool_state[1].sensor[2] != 0:
			flags_text += 'blocked' + "\n"
		else:
			flags_text += 'unblocked' + "\n"
		flags_text += 'switch2: '
		if tool_state[1].sensor[3] != 0:
			flags_text += 'pushed' + "\n"
		else:
			flags_text += 'released' + "\n"
	else:
		flags_text += 'service not running'

	label.config(text = flags_text)

	label.after(250, update_label)

top.title = "Tool State"

label = Label(top, fg = "black", justify=LEFT)
label.pack()
label.after(250,update_label)



gripper1=Button(top,text='gripper1 off',command=lambda: gripper_ctrl1(tool),bg='red')
roller1=Button(top,text='roller1 off',command=lambda: roller_ctrl1(tool),bg='red')
gripper2=Button(top,text='gripper2 off',command=lambda: gripper_ctrl2(tool),bg='red')
roller2=Button(top,text='roller2 off',command=lambda: roller_ctrl2(tool),bg='red')

gripper1.pack()
roller1.pack()
gripper2.pack()
roller2.pack()

top.mainloop()
