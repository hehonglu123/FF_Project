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
def gripper_ctrl(tool):

	if gripper.config('relief')[-1] == 'sunken':
		tool.open()
		gripper.config(relief="raised")
		gripper.configure(bg='red')
		gripper.configure(text='gripper off')

	else:
		tool.close()
		gripper.config(relief="sunken")
		gripper.configure(bg='green')
		gripper.configure(text='gripper on')
	return
def roller_ctrl(tool):

	if roller.config('relief')[-1] == 'sunken':
		tool.setf_param('roll1',RR.VarValue(False,'bool'))
		roller.config(relief="raised")
		roller.configure(bg='red')
		roller.configure(text='roller off')

	else:
		tool.setf_param('roll1',RR.VarValue(True,'bool'))
		roller.config(relief="sunken")
		roller.configure(bg='green')
		roller.configure(text='roller on')
	return


##RR part
def update_label():
	tool_state=state_w.TryGetInValue()
	flags_text = "Tool State Flags:\n\n"
	if tool_state[0]:
		flags_text += 'sensor: '
		if tool_state[1].sensor[0] != 0:
			flags_text += 'blocked' + "\n"
		else:
			flags_text += 'unblocked' + "\n"
		flags_text += 'switch: '
		if tool_state[1].sensor[1] != 0:
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



gripper=Button(top,text='gripper off',command=lambda: gripper_ctrl(tool),bg='red')
roller=Button(top,text='roller off',command=lambda: roller_ctrl(tool),bg='red')


gripper.pack()
roller.pack()

top.mainloop()
