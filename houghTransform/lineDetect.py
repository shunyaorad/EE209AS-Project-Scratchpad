import cv2
import numpy as np

# video_capture = cv2.VideoCapture(0)

# def recordGrayVideo(cap):
#     ret, frame = cap.read()
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     return gray, frame

# while True:
# 	gray, image = recordGrayVideo(video_capture)
# 	edges = cv2.Canny(gray,50,150,apertureSize = 3)

# 	lines = cv2.HoughLines(edges,1,np.pi/180,200)
# 	if len(lines) != 0:
# 		for rho,theta in lines[0]:
# 		    a = np.cos(theta)
# 		    b = np.sin(theta)
# 		    x0 = a*rho
# 		    y0 = b*rho
# 		    x1 = int(x0 + 1000*(-b))
# 		    y1 = int(y0 + 1000*(a))
# 		    x2 = int(x0 - 1000*(-b))
# 		    y2 = int(y0 - 1000*(a))

# 		    cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)

# 	cv2.imshow("Output", image)
# 	if cv2.waitKey(1) & 0xFF == ord('q'):
# 		break

# # When everything is done, release the capture
# video_capture.release()
# cv2.destroyAllWindows()
img = cv2.imread('distortedTag.JPG')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)

lines = cv2.HoughLines(edges,1,np.pi/180,200)
for rho,theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow('result',img)
cv2.waitKey(0)
cv2.destroyAllWindows()