#!/usr/bin/env python

"""
houghCircles.py
---------------

Finds the soccerball using opencv's Hough circle transfrom

Example code at for this library is at the bottom of the file and will execute
if this file is run.

"""
import os, cv2
import numpy as np
from utils import HSVFilter

def houghCircles(img,
    visualize=False,
    setThresholds=False,
    bounds={"upperHSV":np.array([50,255,255]),"lowerHSV":np.array([45,180,30])}):

    """
    # Create blurred img
    #blur = cv2.medianBlur(img, 5)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blur = cv2.medianBlur(hsv, 9)


    # Manually adjust thresholding
    if setThresholds:
        hf = HSVFilter(blur,bounds)
        bounds = hf.createBounds()
        print bounds

    # Filter image
    binary = cv2.inRange(blur, bounds["lowerHSV"],bounds["upperHSV"])
    val = cv2.medianBlur(hsv[:,:,2], 5) 
    grey = cv2.bitwise_and(val,val,mask = binary)  

    """
    

    #***Apply Blur***
    #average
    #blur = cv2.blur(img, (9,9))
    #gaussian
    #blur = cv2.GaussianBlur(img,(9,9),0)
    #median
    blur = cv2.medianBlur(img, 9)
    #bilaterial
    #blur =cv2.bilateralFilter(img,7,11,11)

    # Filter image
    grey = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY) 

    # Apply Hough circles transform
    circles = cv2.HoughCircles(grey,cv2.cv.CV_HOUGH_GRADIENT,1,100,param1=50,param2=50,minRadius=20,maxRadius=250)
    
    if circles is not None:
        #round values to ints
        circles = np.uint16(np.around(circles[0]))
        #TODO:pick correct one
        x,y = circles[0][0],circles[0][1]
        radius =circles[0][2]
    else:
        print "Can't find ball"
        x,y,radius = 100,100,100
 
    # TODO: Visualize
    if visualize:
        bgr = cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)
        #draw all circles and centers
        if circles is not None:
            for i in circles:
                # draw the outer circle
                cv2.circle(bgr,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(bgr,(i[0],i[1]),2,(0,0,255),3)
            #label most likely circle
            cv2.circle(bgr,(i[0],i[1]),i[2],(255,0,0),2)

        cv2.imshow("HoughCircles", bgr)
        cv2.waitKey(0)

    # Pass back describtor
    return (x,y), radius

if __name__ == "__main__":
    """
    # Load Image
    cur_path = os.path.dirname(os.path.realpath(__file__))  #path to script
    rel_path = "../../images/"                              #relative path to img folder
    dir_path = os.path.join(cur_path,rel_path)              #full path to img folder
    img_name = "00_neatoSoccer.png"                         #img name
    img_path = os.path.join(dir_path,img_name)              #img path
    img = cv2.imread(img_path)                              #load from path

    # Run Detector
    #descriptor = houghCircles(img, visualize=True)
    descriptor = houghCircles(img, setThresholds=True, visualize=True)
    print descriptor
    """

    cur_path = os.path.dirname(os.path.realpath(__file__))  #path to script
    rel_path = "../../images/"                              #relative path to img folder
    dir_path = os.path.join(cur_path,rel_path)              #full path to img folder

    for i in range(0,7): 
        img_name = "0" + str(i) + "_neatoSoccer.png" 
        img_path = os.path.join(dir_path,img_name)              #img path
        img = cv2.imread(img_path)                              #load from path

        descriptor = houghCircles(img, visualize=True)

