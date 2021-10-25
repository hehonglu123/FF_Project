#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, yaml, traceback
from tkinter import *
from tkinter import messagebox
import numpy as np


def multisplit(s, delims):
	pos = 0
	for i, c in enumerate(s):
		if c in delims:
			yield s[pos:i]
			pos = i + 1
	yield s[pos:]


top=Tk()
top.title('Testbed Yaml Gui')

with open(r'testbed.yaml') as file:
	testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)

def create_robot_yaml():
	yaml_dict={'bin1_p':list(map(float,filter(None,multisplit(bin1_p.get(),' ')))),
	'bin1_R':list(map(float,filter(None,multisplit(bin1_R.get(),' ')))),
	'bin2_p':list(map(float,filter(None,multisplit(bin2_p.get(),' ')))),
	'bin2_R':list(map(float,filter(None,multisplit(bin2_R.get(),' ')))),
	'place_p':list(map(float,filter(None,multisplit(place_p.get(),' ')))),
	'place_R':list(map(float,filter(None,multisplit(place_R.get(),' ')))),
	'home_p':list(map(float,filter(None,multisplit(home_p.get(),' ')))),
	'home_R':list(map(float,filter(None,multisplit(home_R.get(),' ')))),
	'vision_q':list(map(float,filter(None,multisplit(vision_q.get(),' '))))}
	with open(r'testbed.yaml', 'w') as file:
		yaml.dump(yaml_dict, file)
	messagebox.showinfo(title=None, message='testbed yaml created!')
	return




Label(top, text="Bin1 Position").grid(row=0,column=0)
Label(top, text="Bin1 Orientation").grid(row=0,column=2)
Label(top, text="Bin2 Position").grid(row=1,column=0)
Label(top, text="Bin2 Orientation").grid(row=1,column=2)
Label(top, text="Place Position").grid(row=2,column=0)
Label(top, text="Place Orientation").grid(row=2,column=2)
Label(top, text="Home Position").grid(row=3,column=0)
Label(top, text="Home Orientation").grid(row=3,column=2)
Label(top, text="Vision q").grid(row=4,column=0)

bin1_p = Entry(top)
bin1_R = Entry(top)
bin2_p = Entry(top)
bin2_R = Entry(top)
place_p = Entry(top)
place_R = Entry(top)
home_p = Entry(top)
home_R = Entry(top)
vision_q = Entry(top)

bin1_p.grid(row=0,column=1)
bin1_R.grid(row=0,column=3)
bin2_p.grid(row=1,column=1)
bin2_R.grid(row=1,column=3)
place_p.grid(row=2,column=1)
place_R.grid(row=2,column=3)
home_p.grid(row=3,column=1)
home_R.grid(row=3,column=3)
vision_q.grid(row=4,column=1)

create_file=Button(top,text='Create testbed yaml',command=lambda: create_robot_yaml())
create_file.grid(row=4,column=3)
try:
	bin1_p.insert(0,testbed_yaml['bin1_p'])
except:
	pass
try:
	bin1_R.insert(0,testbed_yaml['bin1_R'])
except:
	pass
try:
	bin2_p.insert(0,testbed_yaml['bin2_p'])
except:
	pass
try:
	bin2_R.insert(0,testbed_yaml['bin2_R'])
except:
	pass
try:
	place_p.insert(0,testbed_yaml['place_p'])
except:
	pass
try:
	place_R.insert(0,testbed_yaml['place_R'])
except:
	pass
try:
	home_p.insert(0,testbed_yaml['home_p'])
except:
	pass
try:
	home_R.insert(0,testbed_yaml['home_R'])
except:
	pass
try:
	vision_q.insert(0,testbed_yaml['vision_q'])
except:
	pass





top.mainloop()
