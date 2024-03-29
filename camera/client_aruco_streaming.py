#Simple example Robot Raconteur webcam client
#This program will show a live streamed image from
#the webcams.  Because Python is a slow language
#the framerate is low...

from RobotRaconteur.Client import *

import time, traceback
import numpy
import cv2
import sys
from cv2 import aruco

def aruco_process(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters =  aruco.DetectorParameters_create()
    # parameters.minMarkerPerimeterRate=0.00001
    # parameters.adaptiveThreshConstant=20
    # # parameters.minMarkerDistanceRate=0.005
    # parameters.adaptiveThreshWinSizeMin=5
    # parameters.adaptiveThreshWinSizeMax=10
    # parameters.adaptiveThreshWinSizeStep=1

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    return frame_markers

def ImageToMat(image):

    frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

    return frame2
current_frame=None


def main():

    url='rr+tcp://pi_fuse:59823?service=camera'
    if (len(sys.argv)>=2):
        url=sys.argv[1]

    #Startup, connect, and pull out the camera from the objref    
    c=RRN.ConnectService(url)

    #Connect the pipe FrameStream to get the PipeEndpoint p
    p=c.frame_stream.Connect(-1)

    #Set the callback for when a new pipe packet is received to the
    #new_frame function
    p.PacketReceivedEvent+=new_frame
    try:
        c.start_streaming()
    except: 
        traceback.print_exc()
        pass

    cv2.namedWindow("Image")

    while True:
        #Just loop resetting the frame
        #This is not ideal but good enough for demonstration

        if (not current_frame is None):
            cv2.imshow("Image",aruco_process(current_frame))
        if cv2.waitKey(50)!=-1:
            break
    cv2.destroyAllWindows()

    p.Close()
    c.stop_streaming()

now=time.time()
#This function is called when a new pipe packet arrives
def new_frame(pipe_ep):
    global current_frame, now
    # print('fps= ', 1/(time.time()-now))
    now=time.time()
    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        
        image=pipe_ep.ReceivePacket()
        #Convert the packet to an image and set the global variable
        current_frame=ImageToMat(image)

        return

if __name__ == '__main__':
    main()
