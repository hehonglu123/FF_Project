#!/usr/bin/env python
#M1k service (simple) for FF Project
#import robotraconteur library
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s

import sys, time, threading, copy, traceback, signal
import numpy as np
from pysmu import Session, LED, Mode, exceptions

minimal_m1k_interface="""
service adi.pysmu.m1k

object m1k_obj
    function void setvoltage(double v)
end object
"""

class m1k(object):
    #initialization
    def __init__(self):
        #start session
        self.session = Session(ignore_dataflow=True, queue_size=100000)
        
        # get first (only?) device
        self.device = self.session.devices[0]
        #define sample rate from default
        self.sample_rate = self.device.default_rate
        #define streaming sample size
        self.sample_size=1
        #initialize default mode to HI_Z
        self.device.channels['A'].mode = Mode.SVMI
        self.device.channels['B'].mode = Mode.HI_Z

        self.session.start(0)

        self.device.channels['A'].constant(0)
        
    def setvoltage(self,v):
        self.device.channels['A'].constant(v)

def main():
    with RR.ServerNodeSetup("M1K_Service_Node", 11111) as node_setup:
        #Register the service type
        RRN.RegisterServiceType(minimal_m1k_interface)

        m1k_obj=m1k()

        #Register the service with object m1k_obj
        RRN.RegisterService("m1k","adi.pysmu.m1k.m1k_obj",m1k_obj)


        #Wait for program exit to quit
        if sys.platform=="linux":
            print("M1k Started, Press ctrl+c to quit")
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        else:
            input("M1k Started, Press enter to quit")

        m1k_obj.device.channels['A'].constant(0)
        m1k_obj.session.end()
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()
    
