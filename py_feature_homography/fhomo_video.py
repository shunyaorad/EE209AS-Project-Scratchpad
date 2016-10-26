import numpy as np
import cv2
  
ESC=27   
camera = cv2.VideoCapture(0)

# The parameter cv2.NORM_HAMMING specifies the distance measurement to be used, 
# in this case, hamming distance. For ORB this is the only option we have for this parameter.
# The second parameter is a boolean  if it is true, the matcher returns only those matches 
# with value (i,j) such that i-th descriptor in set A has j-th descriptor in set B as the 
# best match and vice-versa.
orb = cv2.ORB_create()
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


imgTrainColor=cv2.imread('images/tag_4.jpg')
imgTrainGray = cv2.cvtColor(imgTrainColor, cv2.COLOR_BGR2GRAY)

# the first function is used to get the Key points in the image that we pass 
# as first argument, the second argument is a mask but here we will use none. 
# the second function computes the descriptors of the image using the keypoints. 
# this image shows the key points obtained by ORB
kpTrain = orb.detect(imgTrainGray,None)
kpTrain, desTrain = orb.compute(imgTrainGray, kpTrain)

firsttime=True

while True:
   
    ret, imgCamColor = camera.read()
    imgCamGray = cv2.cvtColor(imgCamColor, cv2.COLOR_BGR2GRAY)

    #  the keypoints and compute the descriptors as before. 
    kpCam = orb.detect(imgCamGray,None)
    kpCam, desCam = orb.compute(imgCamGray, kpCam)

    # we need to delete the matches that correspond to errors, 
    # we do this by setting a distance threshold and obtain the matches points 
    # whose distances are less than the threshold. Here I used as threshold half 
    # of the distance mean across all the array of matching points. 
    matches = bf.match(desCam,desTrain)
    dist = [m.distance for m in matches]

    # Prevent zero division error. Need to modify so that camera dont drop frame
    if len(dist) == 0:
        continue

    thres_dist = (sum(dist) / len(dist)) * 0.5
    matches = [m for m in matches if m.distance < thres_dist]   

    if firsttime==True:
        h1, w1 = imgCamColor.shape[:2]
        h2, w2 = imgTrainColor.shape[:2]
        nWidth = w1+w2
        nHeight = max(h1, h2)
        hdif = (h1-h2)/2
        firsttime=False
       
    result = np.zeros((nHeight, nWidth, 3), np.uint8)
    result[hdif:hdif+h2, :w2] = imgTrainColor
    result[:h1, w2:w1+w2] = imgCamColor

    # for every matching point we get the point in both images 
    #a draw a line between them. prior to this we must create the 
    #result image that is going to have both matching images. 
    for i in range(len(matches)):
        # points on train image
        pt_a=(int(kpTrain[matches[i].trainIdx].pt[0]), int(kpTrain[matches[i].trainIdx].pt[1]+hdif))

        # points on camera
        pt_b=(int(kpCam[matches[i].queryIdx].pt[0]+w2), int(kpCam[matches[i].queryIdx].pt[1]))

        cv2.line(result, pt_a, pt_b, (255, 0, 0))

    if len(matches) > 3:
        print "found!"
    else:
        print "No detection!"

    cv2.imshow('Camara', result)
  
    key = cv2.waitKey(20)                                 
    if key == ESC:
        break

cv2.destroyAllWindows()
camera.release()