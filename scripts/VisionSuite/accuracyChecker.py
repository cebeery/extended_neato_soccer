#!/usr/bin/env python

"""
accuracyChecker.py
---------------

Visualizes and prints accuracy of a vision suite methods from labeled images

"""

import os, yaml, math, cv2
from blobLocator import blobLocator as bl
from colorFilteredCOM import colorFilteredCOM as cf
from houghCircles import houghCircles as hc


class AccuracyChecker(object):
    def __init__(self,dir_path,img_name,lbl_name):
        """
        Open image to be tested and its label 
        """

    	# Extract label
        with open(os.path.join(dir_path,lbl_file), 'r') as f:
            labels = yaml.load(f)
         
        #set label as attribute
        self.lbl_center = tuple(labels[img_name]["location"])
        self.lbl_size 	= labels[img_name]["size"]

        #get image and draw on label
        self.img = cv2.imread(os.path.join(dir_path,img_name))
        self.drawCircle((self.lbl_center),self.lbl_size,(0,0,255))

    def drawCircle(self,location,size,color):
        """
        Draws a circle onto the image of at given (X,Y) location of 
        of size (x,y) and bgr color 
        """
        cv2.circle(self.img,location, size, color, 2)

    def calcErr(self,center,size,failed):
        """
        Calculates the absolute error of given verse labelled size
        and the distance between the given and labelled circle center
        """
        if failed:
            size_error = "no data"
            distance   = "no data"
        else:
            x_error = self.lbl_center[0]-center[0] 
            y_error = self.lbl_center[1]-center[1] 
            distance = int(math.sqrt(x_error**2 + y_error**2))
            size_error = int(math.fabs(self.lbl_size-size)) 

        return [distance,size_error]

    def main(self):
        #Apply visual suite scripts
        bl_center, bl_size, bl_failed = bl(self.img) #simpleBlob detector method
        cf_center, cf_size, cf_failed = cf(self.img) #COM after color filter method
        hc_center, hc_size, hc_failed = hc(self.img) #Hough circles method

        #Calculate error
        bl_error = self.calcErr(bl_center, bl_size, bl_failed)
        cf_error = self.calcErr(cf_center, cf_size, cf_failed)
        hc_error = self.calcErr(hc_center, hc_size, hc_failed)

        #Print Error
        print("Blob Method Error: (Blue)" 
            + "\nDistance: " + str(bl_error[0]) 
            + "\nSize: " + str(bl_error[1])
            + "\n---")
        print("Color Filtered COM Error: (Green)" 
            + "\nDistance: " + str(cf_error[0]) 
            + "\nSize: " + str(cf_error[1])
            + "\n---")
        print("Hough Circles Error: (Cyan)" 
            + "\nDistance: " + str(hc_error[0]) 
            + "\nSize: " + str(hc_error[1])
            + "\n---")

        #Visualize Error
        self.drawCircle((bl_center),bl_size,(255,0,0))
        self.drawCircle((cf_center),cf_size,(0,255,0))
        self.drawCircle((hc_center),hc_size,(255,255,0))

        cv2.imshow('Visualize Error', self.img)
        cv2.waitKey(0) #close on enter key event

if __name__ == "__main__":

    cur_path = os.path.dirname(os.path.realpath(__file__))  #path to script
    rel_path = "../../images/"                              #relative path to img folder
    dir_path = os.path.join(cur_path,rel_path)              #full path to img folder
    lbl_file = "locator_labels.yaml"                        #label file name

    for i in range(0,7): 
        img_name = "0" + str(i) + "_neatoSoccer.png" 
        img_path = os.path.join(dir_path,img_name)              #img path
        img = cv2.imread(img_path)                              #load from path

        checker = AccuracyChecker(dir_path,img_name,lbl_file)   
        checker.main()  

    """
    # File Path Components
    cur_path = os.path.dirname(os.path.realpath(__file__)) 	#path to script
    rel_path = "../../images/"								#relative path to img folder
    dir_path = os.path.join(cur_path,rel_path)				#full path to img folder
    img_name = "05_neatoSoccer.png"									#img name
    lbl_file = "locator_labels.yaml"						#label file name

    # Create and Run instance
    checker = AccuracyChecker(dir_path,img_name,lbl_file)	
    checker.main()			
	"""