#!/usr/bin/python3
from RobotRaconteur.Client import *
import RobotRaconteur as RR
import sys, os, time, argparse, traceback
import numpy as np
import matplotlib.pyplot as plt


try:
	url='rr+tcp://192.168.50.166:11111?service=m1k'
	sub=RRN.SubscribeService(url)
	m1k_obj = sub.GetDefaultClientWait(1)
	m1k_obj.StartSession()

	m1k_obj.setmode('A', 'SVMI')
	m1k_obj.setawgconstant('A',0.)
except:
	traceback.print_exc()
	print('m1k not available')
	pass

samples_wire=sub.SubscribeWire("samples")
m1k_obj.sample_size=100



now=time.time()
m1k_obj.setawgconstant('A',5.)

voltage=[]
current=[]
timestamp=None
# while time.time()-now<10:
# 	sample_packet=samples_wire.TryGetInValue()
# 	if (not sample_packet[0]) or sample_packet[-1]==timestamp:
# 		continue
# 	samples=sample_packet[1]
# 	timestamp=sample_packet[-1]	
# 	for i in range(int(len(samples)/4)):
# 		voltage.append(samples[4*i])
# 		current.append(samples[4*i+1])
samples=m1k_obj.ContRead(10000)

for sample in samples: # calculate average

    voltage.append(sample.A[0])


plt.plot(voltage)
plt.show()
m1k_obj.EndSession()