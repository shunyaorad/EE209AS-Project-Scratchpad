# find contours of tags 

import cv2
import sys
import time

video_capture = cv2.VideoCapture(0)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
th_area = 5000

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

def getPartImageByRect(rect):
    img = cv2.imread(image_dir + image_file, 1)
    return img[rect[0]:rect[1], rect[2]:rect[3]]

outputs = []
rects = []
approxes = []

while True:
	gray, image = recordGrayVideo(video_capture)
	gray = cv2.GaussianBlur(gray, (3, 3), 0)
	# detect edges in the image
	edged = cv2.Canny(gray, 10, 250)
	closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
	(cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	# loop over the contours
	for c in cnts:
		# approximate the contour
		peri = cv2.arcLength(c, True)
		area = cv2.contourArea(c)
		approx = cv2.approxPolyDP(c, 0.02 * peri, True)
		print area
		# if the approximated contour has four points,
		# and bigger than certain area then assume that the
		# contour is a tag
		if len(approx) == 4 and area > th_area:
			cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
			approxes.append(approx)
			rect = getRectByPoints(approx)
			rects.append(rect)

	cv2.imshow("Output", image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()