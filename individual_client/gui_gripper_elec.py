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
	print('rpi gripper service not available')
	pass
try:	
	url='rr+tcp://pi_fuse:11111?service=m1k'
	m1k_obj = RRN.ConnectService(url)
	m1k_obj.StartSession()
	m1k_obj.setmode('A', 'SVMI')
	m1k_obj.setawgconstant('A',0.)
except:
	print('m1k not available')
	pass

top=Tk()
top.title(tool_name)
jobid = None

def gripper_ctrl():


	if gripper.config('relief')[-1] == 'sunken':
		# tool.setf_param('voltage',RR.VarValue(0.,'single'))
		m1k_obj.setawgconstant('A',0.)
		gripper.config(relief="raised")
		gripper.configure(bg='red')
		gripper.configure(text='gripper off')

	else:
		# tool.setf_param('voltage',RR.VarValue(3.5,'single'))
		m1k_obj.setawgconstant('A',2.)
		gripper.config(relief="sunken")
		gripper.configure(bg='green')
		gripper.configure(text='gripper on')
	return

def pin_ctrl():

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

def relay_ctrl():

	if relay.config('relief')[-1] == 'sunken':
		tool.setf_param('relay',RR.VarValue(0,'int8'))
		relay.config(relief="raised")
		relay.configure(bg='red')
		relay.configure(text='relay off')

	else:
		tool.setf_param('relay',RR.VarValue(1,'int8'))
		relay.config(relief="sunken")
		relay.configure(bg='green')
		relay.configure(text='relay on')
	return


label = Label(top, fg = "black", justify=LEFT)
label.pack()


try:
	gripper=Button(top,text='gripper off',command=lambda: gripper_ctrl(),bg='red')
	pin=Button(top,text='pin up',command=lambda: pin_ctrl(),bg='red')
	relay=Button(top,text='relay off',command=lambda: relay_ctrl(),bg='red')


	gripper.pack()
	pin.pack()
	relay.pack()
	top.mainloop()
except:
	m1k_obj.EndSession()
m1k_obj.EndSession()

