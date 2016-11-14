# group circles for each contour
import cv2
import sys
import time
import operator

video_capture = cv2.VideoCapture(0)
# contour parameter
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
th_area = 10000

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

def detectNumberOfCircles(img, maxNumberOfcircles):
	keypoints = detector.detect(img)
	maxNumberOfCircles = len(keypoints)
	# if numberOfCircles > maxNumberOfcircles:
	# 	maxNumberOfcircles = numberOfCircles
	# refresh max number of detected circles every 1 second
	# if (time.time() - lastCircleTime) > 1:
	# 	maxNumberOfcircles = numberOfCircles
	return maxNumberOfcircles

def printNumberOfCircles(numberOfCircles, img):
	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.putText(img, "Tag number: " + str(numberOfCircles),(10,400), font, 2,(255,255,255),2)

detector = cv2.SimpleBlobDetector(params)

# Dictionary to store (y, x, tagFound) coordinates of each tag
inf = 0
dict = {1:(inf, inf, False), 2:(inf, inf, False), 3:(inf, inf, False), \
4:(inf, inf, False), 5:(inf, inf, False)}

largestY = 0

while True:
	# refresh number of tags in the screen
	dict = {1:(0,0, False), 2:(0,0, False), 3:(0,0,False), 4:(0,0,False), 5:(0,0,False)}

	gray, image = recordGrayVideo(video_capture)
	(height, width, channel) = image.shape 
	gray = cv2.GaussianBlur(gray, (3, 3), 0)
	# detect edges in the image
	edged = cv2.Canny(gray, 10, 250)
	closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
	(cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	# loop over the contours
	for c in cnts:
		# maxNumberOfcircles = 0
		# approximate the contour
		peri = cv2.arcLength(c, True)
		area = cv2.contourArea(c)
		approx = cv2.approxPolyDP(c, 0.02 * peri, True)
		# if the approximated contour has four points,
		# and bigger than certain area then assume that the
		# contour is a tag
		if len(approx) == 4 and area > th_area:
			# initialize max number of circles in a contour
			maxNumberOfcircles = 0
			cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
			# get points of the rectangle
			rect = getRectByPoints(approx)
			# get image of the rectangle
			ROI = getPartImageByRect(rect, gray)
			keypoints = detector.detect(ROI)
			numberOfCircles = len(keypoints)
			font = cv2.FONT_HERSHEY_SIMPLEX		

			# This is temporary. Use center of the rectangle as 
			# coordinate of the tag
			x_center = (rect[2] + rect[3]) / 2
			y_center = (rect[0] + rect[1]) / 2
			dict[numberOfCircles] = (y_center, x_center, True)
			cv2.putText(image, str(numberOfCircles) + ": " + \
				str(dict[numberOfCircles][0:2]),(rect[2],rect[0]), font, 1,(255,255,255),2)
			
			# find closest tag by finding the key with the largest y value
			closestTag = max(dict.iteritems(), key=operator.itemgetter(1))[0]
			cv2.putText(image, "closest: " + \
				str(closestTag),(width/3, height * 5/6), font, 1,(255,0,0),2)

	cv2.imshow("Output", image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()