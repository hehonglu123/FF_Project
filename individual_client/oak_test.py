#!/usr/bin/env python3

import cv2
import depthai as dai
import time
# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.createColorCamera()
xoutVideo = pipeline.createXLinkOut()

xoutVideo.setStreamName("video")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setVideoSize(1920, 1080)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
camRgb.video.link(xoutVideo.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

	video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

	while True:
		now=time.time()
		videoIn = video.get()

		# Get BGR frame from NV12 encoded video frame to show with opencv
		# Visualizing the frame on slower hosts might have overhead
		cv2.imshow("video", videoIn.getCvFrame())
		# print(1/(time.time()-now))
		if cv2.waitKey(1) == ord('q'):
			break