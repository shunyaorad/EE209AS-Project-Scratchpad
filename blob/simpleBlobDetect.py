#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import time

video_capture = cv2.VideoCapture(0)

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10
params.maxThreshold = 40

# Filter by Area.
# params.filterByArea = True
# params.minArea = 1500

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.7

#Filter by Convexity
# params.filterByConvexity = True
# params.minConvexity = 0.9
    
# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.01

# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else : 
	detector = cv2.SimpleBlobDetector_create(params)

def recordGrayVideo(cap):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return gray, frame

maxNumberOfcircles = 0

lastTime = time.time()

while True:
	im_gray, im = recordGrayVideo(video_capture)
	# Detect blobs.
	keypoints = detector.detect(im_gray)

	numberOfCircles = len(keypoints)

	if numberOfCircles > maxNumberOfcircles:
		maxNumberOfcircles = numberOfCircles

	# refresh max number of detected circles every 1 second
	if (time.time() - lastTime) > 1:
		maxNumberOfcircles = numberOfCircles

	print(maxNumberOfcircles)


	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
	# the size of the circle corresponds to the size of blob

	im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.putText(im,str(maxNumberOfcircles),(10,500), font, 1,(255,255,255),2)

	# Show blobs
	cv2.imshow("Keypoints", im_with_keypoints)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

video_capture.release()
cv2.destroyAllWindows()