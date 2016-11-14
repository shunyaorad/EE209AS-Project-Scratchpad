#!/usr/bin/python

# works with python version 2.7.6 and openCV 2.4.9

import cv2
import numpy as np;
import time

video_capture = cv2.VideoCapture(0)

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
params.minThreshold = 10
params.maxThreshold = 100
params.filterByCircularity = True
params.minCircularity = 0.8
params.filterByConvexity = True
params.minConvexity = 0.9
params.filterByInertia = True
params.minInertiaRatio = 0.01
detector = cv2.SimpleBlobDetector(params)
maxNumberOfcircles = 0
lastCircleTime = time.time()

def recordGrayVideo(cap):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return gray, frame

def detectNumberOfCircles(img, maxNumberOfcircles):
	keypoints = detector.detect(img)
	numberOfCircles = len(keypoints)
	if numberOfCircles > maxNumberOfcircles:
		maxNumberOfcircles = numberOfCircles
	# refresh max number of detected circles every 1 second
	if (time.time() - lastCircleTime) > 1:
		maxNumberOfcircles = numberOfCircles
	return maxNumberOfcircles

def printNumberOfCircles(numberOfCircles, img):
	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.putText(img, "Tag number: " + str(numberOfCircles),(10,400), font, 2,(255,255,255),2)

while True:
	im_gray, im = recordGrayVideo(video_capture)
	maxNumberOfCircles = detectNumberOfCircles(im_gray, maxNumberOfcircles)
	# draw circle on the blob
	#im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	printNumberOfCircles(maxNumberOfCircles, im)
	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.imshow("Keypoints", im)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

video_capture.release()
cv2.destroyAllWindows()