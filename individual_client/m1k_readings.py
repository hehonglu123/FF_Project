#!/usr/bin/python3
from RobotRaconteur.Client import *
import RobotRaconteur as RR
import sys, os, time, argparse, traceback
import numpy as np
import matplotlib.pyplot as plt


try:
	url='rr+tcp://fusing_laptop:11111?service=m1k'
	sub=RRN.SubscribeService(url)
	m1k_obj = sub.GetDefaultClientWait(1)
	m1k_obj.StartSession()

	m1k_obj.setmode('A', 'SVMI')
	m1k_obj.setawgconstant('A',0.)
except:
	traceback.print_exc()
	print('m1k not available')
	pass


m1k_obj.EndSession()