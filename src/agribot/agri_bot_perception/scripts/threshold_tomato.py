#!/usr/bin/python3

# Standard imports
import cv2
import numpy as np

def empty(a):
    pass

img = cv2.imread("tomato1.png")

while True:
    img_cpy = img
    imgHSV = cv2.cvtColor(img_cpy,cv2.COLOR_BGR2HSV)

  
    #print(h_min,h_max,s_min,s_max,v_min,v_max)

    lower = np.array([0,2,0])
    upper = np.array([0,255,255])

    mask = cv2.inRange(imgHSV,lower,upper)

    imgResult = cv2.bitwise_and(img,img,mask=mask)

    cv2.imshow("HSV",imgHSV)
    cv2.imshow("mask",mask)
    cv2.imshow("imageResult",imgResult)

    ret,thresh = cv2.threshold(mask,127,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        #cX = int(M["m10"] / M["m00"])
        #cY = int(M["m01"] / M["m00"])

        #cv2.circle(img, (cX, cY), 2, (255, 255, 255), -1)
        #cv2.circle(img,(cX,cY),20,(255,0,0),2)
        (x,y),radius = cv2.minEnclosingCircle(c)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(imgResult,center,radius,(0,255,0),2)

        perimeter = cv2.arcLength(c,True)
        #print(perimeter)
        #cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        

        # display the image
        cv2.imshow("Image", imgResult)
        cv2.waitKey(1)
    
