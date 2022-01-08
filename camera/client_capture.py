#Simple example Robot Raconteur webcam client
#This program will show a live streamed image from
#the webcams.  Because Python is a slow language
#the framerate is low...

from RobotRaconteur.Client import *

import cv2, sys, traceback, argparse
import numpy as np


def ImageToMat(image):

    frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

    return frame2

image_consts=None

def main():
    fusing_laptop='192.168.51.116'
    pi_fuse='192.168.51.25'
    robosewclient='192.168.51.61'
    my_laptop='192.168.51.181'

    #Accept the names of the webcams and the nodename from command line
    parser = argparse.ArgumentParser(description="RR plug and play client")

    url='rr+tcp://'+fusing_laptop+':59823?service=camera'

    #Startup, connect, and pull out the camera from the objref    
    cam=RRN.ConnectService(url)

    global image_consts
    image_consts = RRN.GetConstants('com.robotraconteur.image', cam)


    current_frame=ImageToMat(cam.capture_frame())
    cv2.imwrite('vision_check.jpg',current_frame)

if __name__ == '__main__':
    main()
