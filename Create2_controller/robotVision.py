# Simulate robot vision part. Robot motion not incorporated

import cv2
import sys
import time

faceCascade = cv2.CascadeClassifier('classifier_3.xml')
video_capture = cv2.VideoCapture(0)

# Last time that tag is found
lastFoundTime = time.time()

# if the tag is found for the first time
firstTime = True

# last tag's location
lastTag = 0, 0, 0, 0

# Threshold time for using last found tag as current tag
threshTime = 0.8

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
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    return faces

# Find the tag with the largest area
def findLargestTag(tags):
    largestArea = 0
    bx, by, bw, bh = 0, 0, 0, 0
    for (x, w, y, h) in tags:
        area = (x+w) * (y+h)
        if (area > largestArea):
            largestArea = area
            (bx,bw,by,bh) = (x, y, w, h)
    return [bx, bw, by, bh]

# Draw dot in a center of a rectangle
def drawPoint(tag):
    (x, w, y, h) = tag
    cv2.circle(frame,(x + w/2, y + h/2), 10, (0,0,255), -1)

def isTagFound(tags):
    if len(tags) > 0:
        return True
    else:
        return False

'''
Since usually AR tags have two points to be recognized,
average the two points in the AR tag. Current problem of 
jumping dots between two tags still exist.
'''
def averageTag(tag, lastTag):
    (x,w,y,h) = tag
    (lx,lw,ly,lh) = lastTag
    if (x - lx) < w:
        return (x + lx)/2, (w + lw)/2, (y + ly)/2, (h + lh)/2
    else:
        return tag

'''
Draw 3 lines on screen and divide the region into right, left, button.
When tag is at right, robot turns right, and same for left. When tag is located below
buttom line, it stops. Otherwise it moves forward.
'''
def drawLine(screen):
    (height, width, channel) = screen.shape 
    vertLeft = width/3
    vertRight = width*2/3
    bottom = height*4/5
    cv2.line(screen,(vertLeft,0),(vertLeft,height),(255,0,0),5)
    cv2.line(screen,(vertRight,0),(vertRight,height),(255,0,0),5)
    cv2.line(screen,(0,bottom),(width,bottom),(255,0,0),5)
    font = cv2.FONT_HERSHEY_SIMPLEX

def moveRobot(tag, screen):
    (height, width) = screen.shape 
    vertLeft = width/3
    vertRight = width*2/3
    bottom = height*4/5
    (x,w,y,h) = tag
    x = x + w/2
    y = y + h/2
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
    if middle and up:
        print 'Forward'
        bot.drive_straight(15)
    if left:
        print 'Turn Left'
        bot.turn_clockwise(-15)
        time.sleep(1)
        bot.turn_clockwise(0)
    if right:
        print 'Turn Right'
        bot.turn_clockwise(15)
        time.sleep(1)
        bot.turn_clockwise(0)
    if not up:
        print 'Stop'
        bot.drive_straight(0)


while True:
    # Capture video
    gray, frame = recordGrayVideo(video_capture)

    # detect tags
    tags = detectTag(gray, faceCascade)

    # record time previously found tag
    if isTagFound(tags):
        firstTime = False  # not first time to find tag
        lastFoundTime = time.time()
        # find largest tag
        largestTag = findLargestTag(tags)
        tag = averageTag(largestTag, lastTag)
        lastTag = tag
        drawPoint(tag)

    # draw point if tag was found 0.4 sec ago
    if (time.time() - lastFoundTime) < threshTime and not firstTime:
        drawPoint(tag)

    drawLine(frame)
    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()

