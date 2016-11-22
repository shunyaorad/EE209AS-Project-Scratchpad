# Author: Shun Yao
# Email: shunyaorad@gmail.com

# create2 extracts and executes instruction from each tag.
# Final version of the EE209AS project. Tag follower.

import cv2
import sys
import time
import operator
import create2api

# video parameters ########################################
cap = cv2.VideoCapture(0)
Operation = True # Run the program while True
cntsDrawn = False
font = cv2.FONT_HERSHEY_SIMPLEX
screenHeight = 500
screenWidth = 500
vertLeft = screenWidth/3
vertRight = screenWidth*2/3
bottom = screenHeight*4/5
# connect to robot##########################################
# on command line, if command is robot, connect to robot.
# otherwise debug mode without robot connection
robotMode = False
if len(sys.argv) == 1:
	cmd = 'debug'
else:
	cmd = sys.argv[1]
if cmd == 'robot':
	robotMode = True
else:
	robotMode = False
if robotMode:
	bot = create2api.Create2()
	bot.start()
	bot.safe()

timeOfExploration = 5
timeToExploration = 10
# contour parameter####################################################
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
smallest_area = 10000
largest_area = 50000
largestNumberOfCircles = 5

# SimpleBlobDetector parameters########################################
params = cv2.SimpleBlobDetector_Params()
# TODO: Adjust parameters to make sure detecting circles when distorted
params.minThreshold = 10
params.maxThreshold = 100
params.filterByCircularity = True
params.minCircularity = 0.8
params.filterByConvexity = True
params.minConvexity = 0.9
params.filterByInertia = True
params.minInertiaRatio = 0.001
detector = cv2.SimpleBlobDetector(params)
maxNumberOfcircles = 0

# Tag setup ############################################################
class Tag:

	def __init__(self, tagID, x,y, location, actions, nextTagNum, found, executed):
		self.tagID	= tagID
		self.x = x
		self.y = y
		self.location 	= location
		self.actions 	= actions
		self.nextTagNum = nextTagNum
		self.found 		= found
		self.executed = executed

	# reuturn distance of the tag from the robot
	def distance(self):
		# distance from the tag to the bottom center of the screen
		distance = ((self.x - screenWidth/2)**2 + (self.y - screenHeight)**2)**0.5
		distance = int(round(distance))
		return distance

# initialize tags
inf = 0 # inf means that tag distance is farthest
tag1 = Tag(1, inf, inf, "room A", [5, 5], 2, False, False)
tag2 = Tag(2, inf, inf, "room B", [5, 5], 3, False, False)
tag3 = Tag(3, inf, inf, "room C", [5, 5], 4, False, False)
tag4 = Tag(4, inf, inf, "room D", [5, 0, 5], 1, False, False)
allTags = [tag1, tag2, tag3, tag4]
numberOfTags = len(allTags)
pendingTag = None
lastTag = None

# tag record algorithm parameters ########################################
timeFoundLast = 0
delay = 1
distance_th = 100

##########################################################################
def recordGrayVideo(cap):
	ret, frame = cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	return gray, frame

def displayVideo(image):
    global Operation
    cv2.imshow("Output", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        Operation = False
        if robotMode:
            bot.drive_straight(0)
            bot.turn_clockwise(0)
            bot.destroy()
	    cap.release()
	    cv2.destroyAllWindows()

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
		"Distance: %d"%(tag.distance()), (rect[2],rect[0]), font, 1,(255,255,255),2)

def printNearestTag(tag, image):
	if tag == None:
		return
	cv2.putText(image, "closest: " + \
		str(tag.tagID),(screenWidth/3, screenHeight * 5/6), font, 1,(0,0,255),2)

def printCenter(tag, image):
	 cv2.circle(image, (tag.x, tag.y), 10, (0,0,255), -1)

def drawLine(image):
    cv2.line(image,(vertLeft,0),(vertLeft,screenHeight),(255,0,0),5)
    cv2.line(image,(vertRight,0),(vertRight,screenHeight),(255,0,0),5)
    cv2.line(image,(0,bottom),(screenWidth,bottom),(255,0,0),5)

def judgePosition(tag):
	if tag == None:
		return [False, False, False, False]
		print "No tag input for judgePosition"
	x = tag.x
	y = tag.y
	up, left, right, middle = False, False, False, False
	if y < bottom:
		up = True
	else:
		up = False
	if x < vertRight and x > vertLeft:
		middle = True
	elif x < vertLeft:
		left = True
	elif x > vertRight:
		right = True
	return [up, left, right, middle]

# Return array of tags found and nearest tag
def findTags(gray, image):
	global timeFoundLast
	contours = findContour(gray)
	# reset all tag information
	tags = []
	_nearestTag = None
	xx, yy = 0, 0
	# all tags are not found yet
	for i in range(len(allTags)):
		allTags[i].found = False
	# arbitrary set min distance from the robot
	minDistance = screenHeight ** 2 + screenWidth **2
	for c in contours:
		approx, area = approximateCnt(c)
		# find rectangle
		if len(approx) == 4 and area > smallest_area and area < largest_area:
			# draw contours around the rectangle
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
				_nearestTag = tag
			# print tag information
			printTagInfo(tag, rect, image)
			printNearestTag(_nearestTag, image)
			printCenter(tag, image)
	return tags, _nearestTag

# drive robot based on position of the tag
def moveRobot(direction):
	up, left, right, middle = direction
	if not up:
		print 'Stop'
		if robotMode:
			bot.drive_straight(0)
			bot.turn_clockwise(0)
	if middle and up:
		print 'Forward'
		if robotMode:
			bot.drive_straight(15)
	if left and up:
		print 'Turn Left'
		if robotMode:
			bot.turn_clockwise(-15)
	if right and up:
		print 'Turn Right'
		if robotMode:
			bot.turn_clockwise(15)

# execute instrution specified in the tag.
# return nextTag object if found during executing instruction
def executeInstruction(tag):
	nextTag = None
	# restore other tags tag.executed condition
	for i in range(numberOfTags):
		allTags[i].executed = False
	# label that instruction from the tag is executed before
	tag.executed = True
	nextTagID = tag.nextTagNum
	print "Tag: %d"%(tag.tagID)
	actions = tag.actions
	lengthOfAction = len(actions)
	for i in range(lengthOfAction):
		# even entry is time of rotation
		if i % 2 == 0:
			duration = actions[i]
			print "#########rotate %d sec#########"%(duration)
			if duration >= 0:
				nextTag = rotateRobotCW(duration, [nextTagID])
				if nextTag != None:
					break
			else:
				nextTag = rotateRobotCCW(-duration, [nextTagID])
				if nextTag != None:
					break
		# odd entry is time of driving straight
		else:
			duration = actions[i]
			print "#########drive %d sec#########"%(duration)
			if duration >= 0:
				nextTag = driveForward(duration, [nextTagID])
				if nextTag != None:
					break
			else:
				nextTag = driveBackward(-duration, [nextTagID])
				if nextTag != None:
					break
	return nextTag

# TODOL modified the function so that it will take array as input. ie. [1,3,4]
# if only one tagID is givien, ie [3], it will only look for that particiular ID
# if an array of tagID is given, ie [1,3,4], it will look for these IDs and return
# if one of them is found.
def findOneTag(tagID):
	global Operation, screenHeight, screenWidth, timeFoundLast
	tag = None
	gray, image = recordGrayVideo(cap)
	gray = cv2.GaussianBlur(gray, (3, 3), 0)
	drawLine(image)
	targetTag = None
	contours = findContour(gray)
	# Print target tag info
	cv2.putText(image, "Target: " + \
	str(tagID),(screenHeight/2, screenWidth/2), font, 1,(255,0,255),3)
	for c in contours:
		approx, area = approximateCnt(c)
		# find rectangle
		if len(approx) == 4 and area > smallest_area and area < largest_area:
			# draw contours around the rectangle
			cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
			rect = getRectByPoints(approx)
			ROI = getPartImageByRect(rect, gray)
			keypoints = detector.detect(ROI)
			numberOfCircles = len(keypoints)
			if numberOfCircles > numberOfTags or numberOfCircles == 0:
				continue
			x_center, y_center = centerOfRect(rect)
			tag = allTags[numberOfCircles - 1]
			tags.append(tag)
			# Record tag information
			tag.found = True
			tag.x = x_center
			tag.y = y_center
			# print target tag information
			printTagInfo(tag, rect, image)
			printCenter(tag, image)
			# TODO: Indicator when the robot finds the next tag
			# It's not working maybe because when the robot finds next tag it breaks out of the
			# loop immediately
			print "FindOneTag: Found tag: %d"%(tag.tagID)
			print tagID
			if tag.tagID in tagID:
				timeFoundLast = time.time()
				cv2.putText(image, str(tag.tagID) + ": " + \
					"Distance: %d"%(tag.distance()), (rect[2],rect[0]), font, 1,(255,0,255),3)
				cv2.putText(image, str(tag.tagID) + ": " + \
					"Target found!",(screenHeight/2, screenWidth/2), font, 1,(255,0,255),3)
                targetTag = tag
                break
				# TODO: not working for explore()

	cv2.imshow("Output", image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		Operation = False
	return targetTag

# Robot moves forward for specified time
def driveForward(duration, nextTagNum):
	startTime = time.time()
	nextTag = None
	while (time.time() - startTime) < duration:
		nextTag = findOneTag(nextTagNum)
		if nextTag != None:
			print "*******next tag found!!*******"
			if robotMode:
				bot.drive_straight(0)
			break
		print "driving forward"
		if robotMode:
			bot.drive_straight(15)
	if robotMode:
		bot.drive_straight(0)
	return nextTag

# Robot moves backward for specified time
def driveBackward(duration, nextTagNum):
	nextTag = None
	startTime = time.time()
	while (time.time() - startTime) < duration:
		nextTag = findOneTag(nextTagNum)
		if nextTag != None:
			print "*******next tag found!!*******"
			if robotMode:
				bot.drive_straight(0)
			break
		print "driving backward"
		if robotMode:
			bot.drive_straight(-15)
	if robotMode:
		bot.drive_straight(0)
	return nextTag

# Rotate robot clockwise for specified time
def rotateRobotCW(duration, nextTagNum):
	nextTag = None
	startTime = time.time()
	while (time.time() - startTime) < duration:
		nextTag = findOneTag(nextTagNum)
		if nextTag != None:
			print "*******next tag found!!*******"
			if robotMode:
				bot.turn_clockwise(0)
			break
		print "Rotating CW"
		if robotMode:
			bot.turn_clockwise(15)
	if robotMode:
		bot.turn_clockwise(0)
	return nextTag

# Rotate robot counter clockwise for specified time
def rotateRobotCCW(duration, nextTagNum):
	nextTag = None
	startTime = time.time()
	while (time.time() - startTime) < duration:
		gray = cv2.GaussianBlur(gray, (3, 3), 0)
		nextTag = findOneTag(nextTagNum)
		if nextTag != None:
			print "*******next tag found!!*******"
			if robotMode:
				bot.turn_clockwise(0)
			break
		print "Rotating CCW"
		if robotMode:
			bot.turn_clockwise(-15)
	if robotMode:
		bot.turn_clockwise(0)
	return nextTag

# Stop robot for specified duration
def stopRobot(duration):
	if duration == 0:
		moveRobot([False, False, False, False])
		return None
	startTime = time.time()
	while (time.time() - startTime) < duration:
		moveRobot([False, False, False, False])
		gray, image = recordGrayVideo(cap)
		tags, nearestTag = findNearestTag(gray, image)
		drawLine(image)
		printNearestTag(nearestTag, image)
		displayVideo(image)
		if nearestTag != None:
			break
	return nearestTag

def findNearestTag(gray, image):
	global screenHeight, screenWidth, channel, vertLeft, vertRight, bottom
	(screenHeight, screenWidth, channel) = image.shape
	gray = cv2.GaussianBlur(gray, (3, 3), 0)
	# draw lines on screen
	vertLeft = screenWidth/3
	vertRight = screenWidth*2/3
	bottom = screenHeight*4/5
	# find tags on screen
	tags, nearestTag = findTags(gray, image)
	return tags, nearestTag

# Move robot to desired tag
def moveTo(targetTag, image):
	'''
	If the robot is still far away from tag or still can see the tag,
	the robot moves closer toward the tag
	'''
	while targetTag.distance() > distance_th or tag.found == True:
		# Record video so that computer vision works in this loop
		tags, nearestTag = findTags(gray, image)
		direction = judgePosition(targetTag)
		moveRobot(direction)
		#drawLine(image)
		printNearestTag(targetTag, image)
		moveRobot(direction)
		# TODO: Justify the following function
		displayVideo(image)

	cv2.imshow("Output", image)

# Explore the environment for specified time.
def explore(timeOfExploration):
	start = time.time()
	while (time.time() - start) < timeOfExploration:
		print "********** EXPLORATION *****************"
		nextTag = rotateRobotCW(timeOfExploration, [1,2,3,4])
		if nextTag != None:
			print "########## Tag Found By Exploration ##############"
			break
	return nextTag


# Main loop ###########################################################
while Operation:
	gray, image = recordGrayVideo(cap)
	tags, nearestTag, image = findNearestTag(gray, image)
	drawLine(image)
	'''
	move robot according to tag found.
	if tag is not found, stop the robot.
	else move to the nearest one.
	Use last found tag information to drive for delay seconds.
	'''
	# Found tag
	if nearestTag != None:
		print "*************Found Tag****************"
		# lastTag is used to move robot when the tag was found within last delay seconds.
		lastTag = nearestTag
		# pendingTag is used to execute the tag whose instruction was not executed by lost track.
		pendingTag = nearestTag
		# Execute the instruction after robot is close enough to the tag
		if not nearestTag.executed and nearestTag.distance() < 150:
			pendingTag = None
			# while executing instruction, look for next target tag.
			# After finding next tag, move toward the next tag.
			nearestTag = executeInstruction(nearestTag)
			
		# move to the nearest tag if not executing instruction
		if nearestTag.executed:
			printNearestTag(nearestTag, image)
			direction = judgePosition(nearestTag)
			moveRobot(direction)
			displayVideo(image)
		displayVide(image)
	'''
	When the robot moves too close and loses track of nearest
	tag whose instruction hasn't been executed,
	it will execute the pendingTag's instruction.
	'''
	# TODO: check if the following code is valid. Not sure if I checked correctly.
	if nearestTag == None and pendingTag != None and pendingTag.distance() < 150:
		if not pendingTag.executed:
			# Video is displayed during executeInstruction
			nearestTag = executeInstruction(pendingTag)
			printNearestTag(nearestTag, image)
			pendingTag = None
			direction = judgePosition(nearestTag)
			moveRobot(direction)
		pendingTag = None
		# continue to skip displayVideo(image) at the very end of this loop
		continue

	# Did not find tag but last tag was found within delay seconds
	if nearestTag == None and lastTag != None and (time.time() - timeFoundLast) < delay:
		direction = judgePosition(lastTag)
		printNearestTag(lastTag, image)
		moveRobot(direction)
		lastTag = None
		# video is not displayed so that use the displayVideo(image) at the end of this loop

	# No tag found. stop robot for timeToExploration seconds.
	if nearestTag == None and (time.time() - timeFoundLast) < timeToExploration:
		nearestTag = stopRobot(timeToExploration)
		if nearestTag != None:
			direction = judgePosition(nearestTag)
			moveRobot(direction)
			# video is displayed during stopRobot() so continue to skip displayVideo at the end of this loop
			continue

	# TODO: ask for command if it will explore the surrounding for specified time.
	if nearestTag == None and (time.time() - timeFoundLast) > timeToExploration:
		stopRobot(0)
		cmd = raw_input("Tag not found. Explore the surrounding? (y/n)")
		if cmd == 'y':
			nearestTag = explore(timeOfExploration)
			if nearestTag != None:
				direction = judgePosition(nearestTag)
				moveRobot(direction)
			# video is displayed during explore, so continue to skip displayVideo() at the end of this loop
			continue 

		else:
			print "############ No exploration! Give up! ###############"
			if robotMode:
				bot.drive_straight(0)
				bot.turn_clockwise(0)
				bot.destroy()
			Operation = False
			break
	#displayVideo(image)

# When everything is done, release the capture
# cap.release()
# cv2.destroyAllWindows()