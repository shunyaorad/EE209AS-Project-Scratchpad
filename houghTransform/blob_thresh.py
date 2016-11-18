# Test adaptive threshold

import cv2
import numpy as np

# contour parameter####################################################
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
smallest_area = 1000
largest_area = 50000
largestNumberOfCircles = 5

cap = cv2.VideoCapture(0)

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

def findTags(img):
	global timeFoundLast
	contours = findContour(gray)
	# reset all tag information
	tags = []
	nearestTag = None
	xx, yy = 0, 0
	# all tags are not found yet
	for i in range(len(allTags)):
		allTags[i].found = False
	# arbitrary set min distance from the robot
	minDistance = screenHeight ** 2 + screenWidth **2
	for c in contours:
		approx, area = approximateCnt(c)
		if len(approx) < 5 and len(approx) > 3 and area > smallest_area and area < largest_area:
			firstTime = False
			cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
			rect = getRectByPoints(approx)
			ROI = getPartImageByRect(rect, gray)
			keypoints = detector.detect(ROI)
			numberOfCircles = len(keypoints)
			if numberOfCircles > numberOfTags or numberOfCircles == 0:
				continue
			timeFoundLast = time.time()
			x_center, y_center = centerOfRect(rect)
			tag = allTags[numberOfCircles - 1]
			tags.append(tag)
			# Record tag information
			tag.found = True
			tag.x = x_center
			tag.y = y_center
			# Find nearest tag
			distance = tag.distance()
			if distance < minDistance:
				minDistance = distance
				nearestTag = tag
			# print tag information
			printTagInfo(tag, rect, image)
			printNearestTag(nearestTag)
			printCenter(tag)
	return tags, nearestTag
while True:
	
	ret, frame = cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (3, 3), 0)

	th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
	contours = findContour(th)
	tags, nearestTag = findTags(gray)

	cv2.imshow('thresh',th)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()