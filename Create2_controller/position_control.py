# 1st version of tag classifier.

import cv2
import sys
import time

faceCascade = cv2.CascadeClassifier('classifier_3.xml')

video_capture = cv2.VideoCapture(0)

lastFoundTime = time.time()

firstTime = True

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=30,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    print faces

    largestArea = 0
    (bx, bw, by, bh) = (0, 0, 0, 0)
    # Draw a point on the largest found object
    for (x, y, w, h) in faces:
        area = (x+w) * (y+h)
        if (area > largestArea):
        	largestArea = area
        	(bx,by,bw,bh) = (x, y, w, h)

    if len(faces) > 0:
    	lastFoundTime = time.time()
    	cv2.circle(frame,(bx+bw/2,by+bh/2), 10, (0,0,255), -1)
    	lx, lw, ly, lh = bx, bw, by, bh
    	firstTime = False

    elif (time.time() - lastFoundTime) < 0.4 and not firstTime:
    	cv2.circle(frame,(lx+lw/2,ly+lh/2), 10, (0,0,255), -1)

    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()

