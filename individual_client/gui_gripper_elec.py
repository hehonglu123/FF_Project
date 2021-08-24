#!/usr/bin/python3
from RobotRaconteur.Client import *
import RobotRaconteur as RR
import sys, os, time, argparse, traceback
from tkinter import *
from tkinter import messagebox
import numpy as np
  
#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--tool-name",default='elec',type=str)
args, _ = parser.parse_known_args()
tool_name=args.tool_name

#rpi relay
try:
	tool_sub=RRN.SubscribeService('rr+tcp://pi_fuse:22222?service=tool')
	tool=tool_sub.GetDefaultClientWait(1)
except:
	print('rpi relay not available')
	pass


top=Tk()
top.title(tool_name)
jobid = None

def gripper_ctrl(tool):

	if gripper.config('relief')[-1] == 'sunken':
		tool.setf_param('voltage',RR.VarValue(0.,'single'))
		gripper.config(relief="raised")
		gripper.configure(bg='red')
		gripper.configure(text='gripper off')

	else:
		tool.setf_param('voltage',RR.VarValue(3.5,'single'))
		gripper.config(relief="sunken")
		gripper.configure(bg='green')
		gripper.configure(text='gripper on')
	return

def pin_ctrl(tool):

	if pin.config('relief')[-1] == 'sunken':
		tool.open()
		pin.config(relief="raised")
		pin.configure(bg='red')
		pin.configure(text='pin up')

	else:
		tool.close()
		pin.config(relief="sunken")
		pin.configure(bg='green')
		pin.configure(text='pin down')
	return


label = Label(top, fg = "black", justify=LEFT)
label.pack()



gripper=Button(top,text='gripper off',command=lambda: gripper_ctrl(tool),bg='red')
pin=Button(top,text='pin up',command=lambda: pin_ctrl(tool),bg='red')


gripper.pack()
pin.pack()
top.mainloop()

