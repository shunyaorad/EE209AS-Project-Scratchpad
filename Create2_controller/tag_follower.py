'''
Rough draft of the robot motion algorithm.
Robot follows instruction from the last tag to find next tag.
If the robot has not seen a tag previously or can't find the 
next tag within cerrtain time and distance, it will explore
untill it finds any tag. Each tag has information about the 
current area the robot is in.
'''

# import packages#######################################################
import cv2
import sys
import time
import operator
import math

# flags and parameters ##################################################
video_capture = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX
cntsDrawn = False
# contour parameter
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
smallest_area = 10000
largest_area = 50000
largestNumberOfCircles = 5
# Exploration parameters
tagFound = False
tagFoundPrevious = False
timeToExplore = 10
# size of the screen from camera
screenWidth = 500
screenHeight = 500

# Tag setup ############################################################
class Tag:

	def __init__(self, tagID, x,y, location, actions, nextTagNum, found):
		self.tagID	= tagID
		self.x = x
		self.y = y
		self.location 	= location
		self.actions 	= actions
		self.nextTagNum = nextTagNum
		self.found 		= found

	# reuturn distance of the tag from the robot
	def distance():
		# distance from the tag to the bottom center of the screen
		distance = ((self.x - screenWidth/2)**2 + (self.y - screenHeight)**2)**0.5
		distance = int(round(distance))
		return distance

# initialize tags
numberOfTags = 3
inf = 0 # inf means that tag distance is farthest
tag1 = Tag(1, inf, inf, "room A", [2, 10], 2, False)
tag2 = Tag(2, inf, inf, "room B", [5, 10, 5, 10], 3, False)
tag3 = Tag(3, inf, inf, "room C", [5, 10, 5], 1, False)
allTags = [tag1, tag2, tag3]

# obtain instruction to find next tag. Each tag contains an array
# that is a sequence of action i.e. []
def executeInstruction(tag):
	actions = tag.actions
	lengthOfAction = len(actions)
	for i in range(lengthOfAction):
		# even entry is time of rotation
		if i % 2 == 0:
			rotateRobot(actions[i])
		# odd entry is time of driving straight
		else:
			driveRobot(actions[i])

def rotateRobotCW(time):
	print "rotating cw %d seconds"%(time)

def rotateRobotCCW(time):
	print "rotating ccw %d seconds"%(time)

def driveRobot(time):
	print "driving %d seconds"%(time)

def stopRobot(time):
	print "stopping for %d seconds"%(time) 

# Rotate the robot until if finds a tag.
# Return nearest tag if found tag.
# Give up exploration after timeToExplore seconds.
def exploreEnvironment(time, frame):
	global tagFound
	start = time.time()
	while (time.time() - start) < timeToExplore
		rotateRobot(time)
		tagsFound, nearestTag = findTag(frame)
		if nearestTag != None:
			return nearestTag
		else:
			continue
	return nearestTag
	print "Tag not found nearby. Give up!"

# find all the tags and return list of tags found and closest tag
def findTag(frame):
	global tagFound
	nearestTag = None
	gray = cv2.GaussianBlur(frame, (3, 3), 0)
	contours = findContour(gray)
	tagsFound = []
	shortestDistance = 1000 + (screenWidth**2 + screenHeight**2)
	for c in contours:
		approx, area = approximateCnt(c)
		if len(approx) == 4 and area > smallest_area and area < largest_area:
			tagFound = True
			cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
			rect = getRectByPoints(approx)
			ROI = getPartImageByRect(rect, gray)
			keypoints = detector.detect(ROI)
			tagID = len(keypoints)
			# if tagID is out of range, ignore
			if tagID > numberOfTags and tagID == 0:
				continue
			# update tag info		
			x_center, y_center = centerOfRect(rect)
			tag = allTags[tagID-1]
			tag.x = x_center
			tag.y = y_center
			tag.found = True
			# find nearest tag
			if tag.distance() < shortestDistance:
				shortestDistance = tag.distance()
				nearestTag = tag
			tagsFound.append(tag) 
			printTagInfo(tag, rect, image)	
	printClosestTag(nearestTag, image)
	cv2.imshow("Output", image)
	return tagsFound, nearestTag
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

def moveToTag(tag):
	x_dist = math.abs(tag.x - screenWidth / 2)
	y_dist = math.abs(tag.y - screenHeight)

	# warning. not sure if tag.x gets updated each loop
	while tag.x > screenWidth/2 * 1.2:
		rotateRobotCCW(1)
		# update tag info after motion
		findTag(gray)
	while tag.x < screenWidth/2 * 0.8:
		rotateRobotCW(1)
		findTag(gray)
	while y_dist > 50:
		driveRobot(1)

# Vision #######################################################################
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

def printTagInfo(tag, rect, image):
	cv2.putText(image, str(tag.tagID) + ": " + \
		str(tag.distance()),(rect[2],rect[0]), font, 1,(255,255,255),2)

def printClosestTag(tagID):
	cv2.putText(image, "closest: " + \
		str(tag.tagID),(screenWidth/3, screenHeight * 5/6), font, 1,(255,0,0),2)


# Main Loop ############################################################

while True:
	gray, frame = recordGrayVideo(video_capture)

	if !tagFoundPrevious:
		nearestTag = exploreEnvironment(gray)
		moveToTag(nearestTag)
		executeInstruction(nearestTag)
		tagFoundPrevious = True
	else:
		nextTag = findNextTag(gray)
		executeInstruction(nextTag)
