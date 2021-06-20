from RobotRaconteur.Client import *

import time
import numpy as np


url='rr+tcp://localhost:11111?service=m1k'
m1k_obj = RRN.ConnectService(url)

m1k_obj.setmode('A', 'SVMI')
time.sleep(1)
# m1k_obj.read(1000)

m1k_obj.setawgconstant('A',2)
time.sleep(2)

m1k_obj.read(1000)

m1k_obj.setawgconstant('A',3)
time.sleep(2)

m1k_obj.read(1000)
