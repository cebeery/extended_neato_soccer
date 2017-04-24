#!/usr/bin/env python

"""
colorFilteredCOM.py
---------------

Currently contains placeholder code for locating the center and size of a soccer 
ball in an image

Example code at for this library is at the bottom of the file and will execute
if this file is run.

"""

import os, cv2, math
import numpy as np
from utils import ColorFilter

def colorFilteredCOM(img,
    visualize=False,
    setThresholds=False,
    bounds = {"upperBGR":np.array([0,255,75]),"lowerBGR":np.array([0,80,0])}):

    # Manually adjust thresholding
    if setThresholds:
        cf = ColorFilter(img,bounds)
        bounds = cf.createBounds()
        print bounds

    # Filter image
    binary = cv2.inRange(img, bounds["lowerBGR"],bounds["upperBGR"])  

    # Calculate centroid
    moments = cv2.moments(binary)
    if moments['m00'] != 0:
        x,y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
    else:
        print("Can't find ball COM")
        x,y = 0,0

    # Calculate size
    raw_area = cv2.countNonZero(binary)
    est_area = raw_area * 1.4 #increase by ratio of soccer balls' black/white areas 
    radius = math.sqrt(est_area/math.pi)

    # TODO: Visualize
    if visualize:
        pass
        

    # Pass back describtor
    return ( int(x),int(y) ), int(radius)

if __name__ == "__main__":
    # Load Image
    cur_path = os.path.dirname(os.path.realpath(__file__))  #path to script
    rel_path = "../../images/"                              #relative path to img folder
    dir_path = os.path.join(cur_path,rel_path)              #full path to img folder
    img_name = "00_neatoSoccer.png"                         #img name
    img_path = os.path.join(dir_path,img_name)              #img path
    img = cv2.imread(img_path)  

    # Run Detector
    #descriptor = colorFilteredCOM(img)
    descriptor = colorFilteredCOM(img, setThresholds=True, visualize=True)
    print descriptor

