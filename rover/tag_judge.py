# Judge which tag is closer. Rely on edge detection

import cv2
import sys
import time
import operator

video_capture = cv2.VideoCapture(0)
cntsDrawn = False

# contour parameter
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
smallest_area = 10000
largest_area = 50000
largestNumberOfCircles = 5
# Dictionary to store (y, x, tagFound) coordinates of each tag
# initial coordinate is (0,0,False)
inf = 0
dict = {1:(inf, inf, False), 2:(inf, inf, False), 3:(inf, inf, False), \
4:(inf, inf, False), 5:(inf, inf, False)}
largestY = 0
lastRects = []

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

def getRectByPoints(points):
    # prepare simple array 
    points = list(map(lambda x: x[0], points))

    points = sorted(points, key=lambda x:x[1])
    top_points = sorted(points[:2], key=lambda x:x[0])
    bottom_points = sorted(points[2:4], key=lambda x:x[0])
    points = top_points + bottom_points

    left = min(points[0][0], points[2][0])
    right = max(points[1][0], points[3][0])
    top = min(points[0][1], points[1][1])
    bottom = max(points[2][1], points[3][1])
    return (top, bottom, left, right)

def getPartImageByRect(rect, img):
    return img[rect[0]:rect[1], rect[2]:rect[3]]

def detectNumberOfCircles(img):
	keypoints = detector.detect(img)
	numberOfCircles = len(keypoints)
	return numberOfCircles

def findContour(img):
	edged = cv2.Canny(img, 10, 250)
	closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
	(cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	return cnts

def approximateCnt(contour):
	peri = cv2.arcLength(contour, True)
	area = cv2.contourArea(contour)
	approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
	return approx, area

def centerOfRect(rect):
	x = (rect[2] + rect[3]) / 2
	y = (rect[0] + rect[1]) / 2
	return x, y

def printTagInfo(numberOfCircles, rect, image):
	cv2.putText(image, str(numberOfCircles) + ": " + \
		str(dict[numberOfCircles][0:2]),(rect[2],rect[0]), font, 1,(255,255,255),2)

def printClosestTag(tag, image):
	(height, width, channel) = image.shape 
	cv2.putText(image, "closest: " + \
		str(tag),(width/3, height * 5/6), font, 1,(255,0,0),2)


while True:
	# refresh number of tags in the screen
	tagsFound = [False, False, False, False, False, False]
	dict = {1:(0,0, False), 2:(0,0, False), 3:(0,0,False), 4:(0,0,False), 5:(0,0,False)}

	gray, image = recordGrayVideo(video_capture)
	(height, width, channel) = image.shape 
	gray = cv2.GaussianBlur(gray, (3, 3), 0)
	contours = findContour(gray)

	for c in contours:
		approx, area = approximateCnt(c)
		if len(approx) == 4 and area > smallest_area and area < largest_area:
			firstTime = False
			lastFoundTime = time.time()
			lastRects.append(approx)
			cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
			rect = getRectByPoints(approx)
			ROI = getPartImageByRect(rect, gray)
			keypoints = detector.detect(ROI)
			numberOfCircles = len(keypoints)
			font = cv2.FONT_HERSHEY_SIMPLEX		
			x_center, y_center = centerOfRect(rect)
			dict[numberOfCircles] = (y_center, x_center, True)
			closestTag = max(dict.iteritems(), key=operator.itemgetter(1))[0]
			printTagInfo(numberOfCircles, rect, image)
			printClosestTag(closestTag, image)
			if numberOfCircles > largestNumberOfCircles:
				continue
			tagsFound[numberOfCircles] = True

	cv2.imshow("Output", image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()