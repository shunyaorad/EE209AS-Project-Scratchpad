# 1st version of tag following robot
import cv2
import sys
import time
import create2api

bot = create2api.Create2()
bot.start()
bot.safe()

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
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
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
    cv2.circle(frame,(x+w/2,y+h/2), 10, (0,0,255), -1)

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
    if (x - lx) < (w+lw):
        return (x+lx)/2, (w+lw)/2, (y+ly)/2, (h+lh)/2
    else:
        return tag

def drawLine(screen):
    (height, width, channel) = screen.shape 
    vertLeft = width/3
    vertRight = width*2/3
    bottom = height*3/4
    cv2.line(screen,(vertLeft,0),(vertLeft,height),(255,0,0),5)
    cv2.line(screen,(vertRight,0),(vertRight,height),(255,0,0),5)
    cv2.line(screen,(0,bottom),(width,bottom),(255,0,0),5)
    font = cv2.FONT_HERSHEY_SIMPLEX

def moveRobot(tag, screen):
    (height, width) = screen.shape 
    vertLeft = width/3
    vertRight = width*2/3
    bottom = height*3/4
    (x,w,y,h) = tag
    x = x+w/2
    y = y+h/2
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
        bot.drive_straight(100)
    if left:
        print 'Turn Left'
        bot.turn_clockwise(-100)
    if right:
        print 'Turn Right'
        bot.turn_clockwise(100)
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
    if isTagFound(tags):
        largestTag = findLargestTag(tags)
        tag = averageTag(largestTag, lastTag)
        lastTag = tag
        drawPoint(tag)
        moveRobot(tag, gray)

    # draw point if tag was found 0.4 sec ago
    elif (time.time() - lastFoundTime) < 0.8 and not firstTime:
        drawPoint(tag)
        moveRobot(tag, gray)

    else:
        print 'Stop'
        bot.drive_straight(0)

    drawLine(frame)
    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        bot.destory()
        break
# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()

