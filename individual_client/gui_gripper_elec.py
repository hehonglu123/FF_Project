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
	tool_sub=RRN.SubscribeService('rr+tcp://192.168.50.115:22222?service=tool')
	tool=tool_sub.GetDefaultClientWait(1)
except:
	print('rpi relay not available')
	pass
try:
	m1k_obj = RRN.ConnectService('rr+tcp://192.168.50.166:11111?service=m1k')
	m1k_obj.StartSession()

	m1k_obj.setmode('A', 'SVMI')
	m1k_obj.setawgconstant('A',0.)
except:
	print('m1k not available')
	pass

top=Tk()
top.title(tool_name)
jobid = None

def gripper_ctrl(m1k_obj):

	if gripper.config('relief')[-1] == 'sunken':
		m1k_obj.setawgconstant('A',0.)
		# tool.setf_param('elec',RR.VarValue(False,'bool'))
		gripper.config(relief="raised")
		gripper.configure(bg='red')
		gripper.configure(text='gripper off')

	else:
		m1k_obj.setawgconstant('A',3.5)
		# tool.setf_param('elec',RR.VarValue(True,'bool'))
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



gripper=Button(top,text='gripper off',command=lambda: gripper_ctrl(m1k_obj),bg='red')
pin=Button(top,text='pin up',command=lambda: pin_ctrl(tool),bg='red')


gripper.pack()
pin.pack()
top.mainloop()

m1k_obj.EndSession()