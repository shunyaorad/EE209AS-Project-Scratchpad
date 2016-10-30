# 1st version of tag classifier.

import cv2
import sys

faceCascade = cv2.CascadeClassifier('classifier_3.xml')

video_capture = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=30,
        minSize=(20, 20),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )

    if len(faces) > 0:
        print 'found!'
    else:
        print 'NOOOOOO!'

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()
