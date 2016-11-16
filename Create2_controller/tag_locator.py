# locate a tag in a screen and tells its location by number

import cv2
import sys
import time
from math import *

faceCascade = cv2.CascadeClassifier('classifier_3.xml')
video_capture = cv2.VideoCapture(0)
lastFoundTime = time.time()
firstTime = True
lastTag = 0, 0, 0, 0

def recordGrayVideo(cap):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return gray, frame

def detectTag(img, cascade):
    faces = cascade.detectMultiScale(
        img,
        scaleFactor=1.2,
        minNeighbors=30,
        minSize=(30, 30),
       # flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    return faces

def findLargestTag(tags):
    largestArea = 0
    bx, by, bw, bh = 0, 0, 0, 0
    for (x, w, y, h) in tags:
        area = (x+w) * (y+h)
        if (area > largestArea):
            largestArea = area
            (bx,bw,by,bh) = (x, y, w, h)
    return [bx, bw, by, bh]

def drawPoint(tag):
    (x, w, y, h) = tag
    cv2.circle(frame, (floor(x+w//2), floor(y+h//2)), 10, (0,0,255), -1)

def isTagFound(tags):
    if len(tags) > 0:
        return True
    else:
        return False

# Since usually AR tags have two points to be recognized
# Average the two points in the AR tag
def averageTag(tag, lastTag):
    (x,w,y,h) = tag
    (lx,lw,ly,lh) = lastTag
    if abs(x - lx) < abs(w-lw):
        return (x+lx)/2, (w+lw)/2, (y+ly)/2, (h+lh)/2
    else:
        return tag

def drawLine(screen):
    (height, width, channel) = screen.shape 
    midX = width//2
    midY = height//2
    cv2.line(screen,(midX,0),(midX,height),(255,0,0),5)
    cv2.line(screen,(0,midY),(width,midY),(255,0,0),5)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(screen,'1',(midX//3,midY//2+30), font, 4,(255,255,255),2)
    cv2.putText(screen,'2',(midX//3+330,midY//2+30), font, 4,(255,255,255),2)
    cv2.putText(screen,'3',(midX//3,midY//2+280), font, 4,(255,255,255),2)
    cv2.putText(screen,'4',(midX//3+330,midY//2+280), font, 4,(255,255,255),2)


def checkMarkerPos(tag, screen):
    (height, width) = screen.shape 
    midX = width/2
    midY = height/2
    (x,w,y,h) = tag
    if y+h/2 < midY:
        up = True
    else:
        up = False
    if x+w/2 < midX:
        left = True
    else:
        left = False
    if up and left:
        print('1')
    if up and not left:
        print('2')
    if not up and left:
        print('3')
    if not up and not left:
        print('4')


while True:
    # Capture video
    gray, frame = recordGrayVideo(video_capture)

    # detect tags
    tags = detectTag(gray, faceCascade)

    # record time previously found tag
    if isTagFound(tags):
        firstTime = False  # not first time to find tag
        lastFoundTime = time.time()
        largestTag = findLargestTag(tags)
        tag = averageTag(largestTag, lastTag)
        lastTag = tag
        drawPoint(tag)
        checkMarkerPos(tag, gray)

    # draw point if tag was found 0.4 sec ago
    elif (time.time() - lastFoundTime) < 0.6 and not firstTime:
        drawPoint(tag)
        checkMarkerPos(tag, gray)

    else:
        print('None')

    drawLine(frame)
    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()

