#!/usr/bin/env python

"""
accuracyChecker.py
---------------

Visualizes and prints accuracy of a vision suite options from labeled images

Example code at for this library is at the bottom of the file and will execute
if this file is run.

"""

import os, yaml, cv2

class AccuracyChecker(object):
    def __init__(self,dir_path,img_name,lbl_name):

    	 # Extract label
         with open(os.path.join(dir_path,lbl_file), 'r') as f:
        	labels = yaml.load(f)
        
         self.lbl_center = tuple(labels[img_name]["location"])
         self.lbl_size 	= labels[img_name]["size"]

         self.img = cv2.imread(os.path.join(dir_path,img_name))
         self.drawLabel()

    def drawCircle(self,location,size,color):
        cv2.circle(self.img,location, size, color, 2)

    def drawLabel(self):
        self.drawCircle((self.lbl_center),self.lbl_size,(0,0,255))


    def main(self):
        #Apply visual suite scripts

        #Calculate error

        #Print Error

        #Visualize Error
        cv2.imshow('Visualize Error', self.img)
        cv2.waitKey(0) #close on enter key event

if __name__ == "__main__":

    # File Path Components
    cur_path = os.path.dirname(os.path.realpath(__file__)) 	#path to script
    rel_path = "../../images/"								#relative path to img folder
    dir_path = os.path.join(cur_path,rel_path)				#full path to img folder
    img_name = "test.jpg"									#img name
    lbl_file = "locator_labels.yaml"						#label file name

    # Create and Run instance
    checker = AccuracyChecker(dir_path,img_name,lbl_file)	
    checker.main()			
	