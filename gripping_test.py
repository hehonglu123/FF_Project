#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback
from qpsolvers import solve_qp
import numpy as np
from importlib import import_module
sys.path.append('../toolbox')
from vel_emulate_sub import EmulatedVelocityControl
sys.path.append('../toolbox')
from general_robotics_toolbox import *    


def normalize_dq(q):
	q[:-1]=0.5*q[:-1]/(np.linalg.norm(q[:-1])) 
	return q   

