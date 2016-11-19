# Experiment with opencv to display video in loop within while loop

import cv2
import sys
import time

cap = cv2.VideoCapture(0)
flag = False
Condition = True

def recordGrayVideo(cap):
	ret, frame = cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	return gray, frame

def deepInsideA(start):
	global Condition
	while (time.time() - start) < 2:
		gray, image = recordGrayVideo(cap)
		cv2.imshow("Output", image)
		print "AAA!!!"
		if cv2.waitKey(1) & 0xFF == ord('q'):
			Condition = False
			break

def deepInsideB(start):
	global Condition
	while (time.time() - start) < 2:
		gray, image = recordGrayVideo(cap)
		cv2.imshow("Output", image)
		print "BBB!!!"
		if cv2.waitKey(1) & 0xFF == ord('q'):
			Condition = False
			break

def inside():
	global flag
	flag = False
	start = time.time()
	deepInsideA(start)

def outside():
	global Condition
	global flag
	flag = True
	start = time.time()
	deepInsideB(start)

while Condition:
	# gray, image = recordGrayVideo(cap)
	# cv2.imshow("Output", image)
	if flag:
		inside()
	else:
		outside()

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
