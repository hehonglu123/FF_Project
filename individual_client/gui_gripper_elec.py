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



url='rr+tcp://192.168.50.166:11111?service=m1k'
m1k_obj = RRN.ConnectService(url)
m1k_obj.StartSession()

m1k_obj.setmode('A', 'SVMI')
m1k_obj.setawgconstant('A',0.)

top=Tk()
top.title(tool_name)
jobid = None
def gripper_ctrl1(m1k_obj):

	if gripper1.config('relief')[-1] == 'sunken':
		m1k_obj.setawgconstant('A',0.)
		gripper1.config(relief="raised")
		gripper1.configure(bg='red')
		gripper1.configure(text='gripper off')

	else:
		m1k_obj.setawgconstant('A',5.)
		gripper1.config(relief="sunken")
		gripper1.configure(bg='green')
		gripper1.configure(text='gripper on')
	return



label = Label(top, fg = "black", justify=LEFT)
label.pack()



gripper1=Button(top,text='gripper1 off',command=lambda: gripper_ctrl1(m1k_obj),bg='red')


gripper1.pack()

top.mainloop()

m1k_obj.EndSession()