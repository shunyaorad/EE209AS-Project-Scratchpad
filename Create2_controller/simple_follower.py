# create2 move to the nearest tag

import cv2
import sys
import time
import operator
import create2api

video_capture = cv2.VideoCapture(0)
cntsDrawn = False
font = cv2.FONT_HERSHEY_SIMPLEX

bot = create2api.Create2()
bot.start()
bot.safe()

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

def printCenter(x, y):
	 cv2.circle(image, (x, y), 10, (0,0,255), -1)

def drawLine():
    cv2.line(image,(vertLeft,0),(vertLeft,height),(255,0,0),5)
    cv2.line(image,(vertRight,0),(vertRight,height),(255,0,0),5)
    cv2.line(image,(0,bottom),(width,bottom),(255,0,0),5)
    font = cv2.FONT_HERSHEY_SIMPLEX

def judgePosition(rect):
	x, y = centerOfRect(rect)
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

def findTags(img):
	contours = findContour(gray)
	tags = []
	nearestTag = None
	xx, yy = 0, 0
	for c in contours:
		approx, area = approximateCnt(c)
		if len(approx) < 5 and len(approx) > 3 and area > smallest_area and area < largest_area:
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
			if numberOfCircles > largestNumberOfCircles:
				continue
			printTagInfo(numberOfCircles, rect, image)
			printClosestTag(closestTag, image)
			printCenter(x_center, y_center)
			tags.append([x_center,y_center])
			# find nearest tag
			if y_center > yy:
				yy = y_center
				nearestTag = rect

	return tags, nearestTag

def moveRobot(direction):
	up, left, right, middle = direction
	if not up:
		print 'Stop'
		bot.drive_straight(0)
		bot.turn_clockwise(0)
	if middle and up:
		print 'Forward'
		bot.drive_straight(15)
	if left and up:
		print 'Turn Left'
		bot.turn_clockwise(-15)
	if right and up:
		print 'Turn Right'
		bot.turn_clockwise(15)

while True:
	gray, image = recordGrayVideo(video_capture)
	(height, width, channel) = image.shape 
	# needed for determining position of the tag and draw lines on screen
	vertLeft = width/3
	vertRight = width*2/3
	bottom = height*4/5
	drawLine()
	gray = cv2.GaussianBlur(gray, (3, 3), 0)
	contours = findContour(gray)
	tags, nearestTag = findTags(gray)
	if nearestTag == None:
		moveRobot([False, False, False, False])
	else:
		direction = judgePosition(nearestTag)
		moveRobot(direction)

	cv2.imshow("Output", image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		bot.drive_straight(0)
		bot.turn_clockwise(0)
		bot.destroy()
		break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()