#!/usr/bin/env python

""" 
utils.py
-----

Adds visualization and boundery setting helper funtions for visual suite

"""

import cv2, math
import numpy as np

class ColorFilter(object):
    """ 
    ColorFilter is a Python object that can visualize and set color filter
    bounding values on an image
    """
    def __init__(self, img, bounds={"upperBGR":np.array([255,255,255]),"lowerBGR":np.array([0,0,0])}):
        """ Initialize the ball tracker """   

        #set initial boundry values
        self.lower = bounds["lowerBGR"]
        self.upper = bounds["upperBGR"]
        
        #create images     
        self.IMG = img 

    def createBounds(self):
        """Create color bound gui controls"""

        #visualize raw
        cv2.namedWindow('raw_window')
        cv2.imshow('raw_window', self.IMG)

        #set callback for show color under mouse
        cv2.setMouseCallback('raw_window', self.process_mouse_event)
        print "Press enter to continue to thresholding"
        cv2.waitKey(0)

        #create track bars  
        cv2.namedWindow('threshold_image')
        cv2.createTrackbar('blue lower bound', 'threshold_image', self.lower[0], 255, self.set_blue_lower_bound)
        cv2.createTrackbar('blue upper bound', 'threshold_image', self.upper[0], 255, self.set_blue_upper_bound)
        cv2.createTrackbar('green lower bound', 'threshold_image', self.lower[1], 255, self.set_green_lower_bound)
        cv2.createTrackbar('green upper bound', 'threshold_image', self.upper[1], 255, self.set_green_upper_bound)
        cv2.createTrackbar('red lower bound', 'threshold_image', self.lower[2], 255, self.set_red_lower_bound)
        cv2.createTrackbar('red upper bound', 'threshold_image', self.upper[2], 255, self.set_red_upper_bound)

        #visualize manual bounds updating 
        print ("Hover over raw image to see color values")
        print ("Click close x to close window and submit most recent trackbar values")
        while cv2.getWindowProperty('threshold_image', cv2.WND_PROP_AUTOSIZE) >= 0:
            binary = cv2.inRange(self.IMG, self.lower, self.upper)  
            cv2.imshow("threshold_image", binary)
            cv2.waitKey(1) 

        #cleanup and send back results
        cv2.destroyAllWindows() 
        return {"upperBGR":self.upper,"lowerBGR":self.lower}

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((100,500,3))
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.IMG[y,x,0], self.IMG[y,x,1], self.IMG[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def set_blue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue upper bound """
        self.upper[0] = val

    def set_blue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue lower bound """
        self.lower[0] = val

    def set_green_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the green upper bound """
        self.upper[1] = val

    def set_green_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the green lower bound """
        self.lower[1] = val

    def set_red_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red upper bound """
        self.upper[2] = val

    def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.lower[2] = val

class HSVFilter(object):
    """ 
    HVSFilter is a Python object that can visualize and set hsv filter
    bounding values on an image
    """
    def __init__(self, img, bounds={"upperHSV":np.array([255,255,255]),"lowerHSV":np.array([0,0,0])}):
        """ Initialize the ball tracker """   

        #set initial boundry values
        self.lower = bounds["lowerHSV"]
        self.upper = bounds["upperHSV"]
        
        #create images     
        self.IMG = img 

    def createBounds(self):
        """Create color bound gui controls"""

        #visualize raw
        cv2.namedWindow('raw_window')
        cv2.imshow('raw_window', self.IMG)

        #set callback for show color under mouse
        cv2.setMouseCallback('raw_window', self.process_mouse_event)
        print "Press enter to continue to thresholding"
        cv2.waitKey(0)

        #create track bars  
        cv2.namedWindow('threshold_image')
        cv2.createTrackbar('hue lower bound', 'threshold_image', self.lower[0], 255, self.set_hue_lower_bound)
        cv2.createTrackbar('hue upper bound', 'threshold_image', self.upper[0], 255, self.set_hue_upper_bound)
        cv2.createTrackbar('saturation lower bound', 'threshold_image', self.lower[1], 255, self.set_saturation_lower_bound)
        cv2.createTrackbar('saturation upper bound', 'threshold_image', self.upper[1], 255, self.set_saturation_upper_bound)
        cv2.createTrackbar('value lower bound', 'threshold_image', self.lower[2], 255, self.set_value_lower_bound)
        cv2.createTrackbar('value upper bound', 'threshold_image', self.upper[2], 255, self.set_value_upper_bound)

        #visualize manual bounds updating 
        print ("Hover over raw image to see color values")
        print ("Click close x to close window and submit most recent trackbar values")
        while cv2.getWindowProperty('threshold_image', cv2.WND_PROP_AUTOSIZE) >= 0:
            binary = cv2.inRange(self.IMG, self.lower, self.upper)  
            cv2.imshow("threshold_image", binary)
            cv2.waitKey(1) 

        #cleanup and send back results
        cv2.destroyAllWindows() 
        return {"upperHSV":self.upper,"lowerHSV":self.lower}

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((100,500,3))
        cv2.putText(image_info_window,
                    'Color (h=%d,s=%d,v=%d)' % (self.IMG[y,x,0], self.IMG[y,x,1], self.IMG[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def set_hue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the hue upper bound """
        self.upper[0] = val

    def set_hue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the hue lower bound """
        self.lower[0] = val

    def set_saturation_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the saturation upper bound """
        self.upper[1] = val

    def set_saturation_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the saturation lower bound """
        self.lower[1] = val

    def set_value_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the value upper bound """
        self.upper[2] = val

    def set_value_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the value lower bound """
        self.lower[2] = val


if __name__ == '__main__':
    pass