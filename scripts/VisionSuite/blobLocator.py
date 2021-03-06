#!/usr/bin/env python

"""
blobLocator.py
---------------

Currently contains placeholder code for locating the center and size of a soccer 
ball in an image

Example code at for this library is at the bottom of the file and will execute
if this file is run.

"""
import os, cv2, math
import numpy as np
from utils import ColorFilter

def blobLocator(img,
    visualize=False,
    setThresholds=False,
    bounds = {}):

    #Assume failure
    error = True

    # Manually adjust thresholding
    if setThresholds:
        pass

    # Filter image
    #grey = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)  
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #blur = cv2.blur(hsv[:,:,0],(71,71))
    #equ = cv2.equalizeHist(blur)
    #mask = cv2.inRange(equ, 220, 255) 

    #grey= cv2.bitwise_and(hsv[:,:,2],hsv[:,:,2],mask = mask) 
    #grey = cv2.bilateralFilter(grey,11,5,5)

    #grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  
    grey = hsv[:,:,0]
    grey = cv2.medianBlur(grey,11)

    # Create SimpleBlobDetector
    params = cv2.SimpleBlobDetector_Params()

    params.minDistBetweenBlobs = 1

    params.minThreshold = 5
    params.maxThreshold = 250
    params.thresholdStep = 1

    params.filterByArea = True
    params.minArea = int(20**2*math.pi)
    params.maxArea = int(200**2*math.pi)

    ## Filter by Circularity
    #params.filterByCircularity = True
    #params.minCircularity = 0.1
    #params.maxCircularity = 0.9

    ## Filter by Convexity
    #params.filterByConvexity = True
    #params.minConvexity = 0.87
 
    ## Filter by Inertia
    #params.filterByInertia = True
    #params.minInertiaRatio = 0.01

    params.filterByColor = True
    params.blobColor = 255

    sbd = cv2.SimpleBlobDetector(params)


    # Apply detector
    blobs = sbd.detect(grey)

    # TODO: Visualize
    if visualize:
        bgr = cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)
        for blob in blobs:
            cv2.circle(bgr,(int(blob.pt[0]),int(blob.pt[1])),int(blob.size), (0,0,255), 1)

        cv2.imshow("blobDetector", bgr)
        cv2.waitKey(0)

    if error:
        error = True
        print "Blob Detector: can't find ball"
        x,y,radius = 0,0,50

    # Pass back describtor    
    return (x,y), radius, error


if __name__ == "__main__":
    # Load Image
    cur_path = os.path.dirname(os.path.realpath(__file__))  #path to script
    rel_path = "../../images/"                              #relative path to img folder
    dir_path = os.path.join(cur_path,rel_path)              #full path to img folder
    img_name = "00_neatoSoccer.png"                         #img name
    img_path = os.path.join(dir_path,img_name)              #img path
    img = cv2.imread(img_path)                              #load from path

    # Run Detector
    descriptor = blobLocator(img, visualize=True)
    #descriptor = blobLocator(img, setThresholds=True, visualize=True)
    #print descriptor
