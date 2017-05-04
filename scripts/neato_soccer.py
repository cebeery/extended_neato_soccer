#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy, cv2, math
import numpy as np
from cv_bridge import CvBridge
from neato_node.msg import Bump
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from NeuralNet.balldetector import BallDetector
from VisionSuite.colorFilteredCOM import colorFilteredCOM
from VisionSuite.blobLocator import blobLocator
from VisionSuite.houghCircles import houghCircles


class NeatoSoccerPlayer(object):
    """ 
    The NeatoSoccerPlayer is a Python object that encompasses a ROS node 
    that can process images from the camera and search for a ball within.
    The node will issue motor commands to align the center of the camera's
    field of view with the ball and then move forward  to hit the ball  
    """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """

        # Setup states
        #states = determineBall, alignNeato
        self.state = self.determineBall
        self.transition = True

        # Setup Neural Net Class Instance
        self.nn = BallDetector()
        print "ready to rumble"

        # Setup raw camera visualization
        self.img = None                 #the latest image from the camera
        self.bridge = CvBridge()        #used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window') #create window for live video stream

        # Setup ROS Node
        print "Trying to connect to Neato"
        rospy.init_node('neato_soccer')
        self.rate = rospy.Rate(10)
        rospy.Subscriber(image_topic, Image, self.processImage)
        rospy.Subscriber("/bump", Bump, self.processBump)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd = Twist()

    def processImage(self, msg):
        """
        Process image messages from ROS and stash them in an attribute
        called img for subsequent processing
        """
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def processBump(self, msg):
        """ End node if ball kicked or neato runs into something """
        if msg.leftFront or msg.rightFront or msg.leftSide or msg.rightSide:
            print "Crash!"
            rospy.signal_shutdown("Neato hit object")

    def determineBall(self):
        """
        Determines if there is a soccer ball in the view currently being processed
        using a neural net; initiates rotation to look for ball, if needed
        At Start    > ensure no neato motion
        Transition 1: condition(ball found) > alignNeato state
        """

        # init change state flag
        ball = None

        # perform state funtionally
        if self.transition:
            print "Transitioning to determineBall state"
            self.cmd = Twist()
            self.transition = False
        else:
            print "Looking for ball"
            ball = self.nn.classify(self.img)

        # change state if needed
        if ball:
            print "Ball exists"
            self.state = self.alignNeato
            self.transition = True
        elif ball is not None:
            print "Where ball? Want ball!" 
            self.cmd.angular.z = 0.5

    def alignNeato(self):
        """
        Determines the location of the ball within the current image using one of multiple
        image processing options from the Vision Suite. If ball is in center of frame, 
        neato drives forward. If ball is not within center threshold, neato rotates at a
        speed proportional to offset line up with ball and drives forward slower with greater 
        misalignment  
        At Start    > ensure no neato motion
        Transition 1: condition(no ball) > determineBall state
        """

        # init change state flag
        error = False

        # perform state funtionally
        if self.transition:
            print "Transitioning to alignNeato state"
            self.cmd = Twist()
            self.transition = False
        else:
            #use on of vision suite methods
            location,_,error = colorFilteredCOM(self.img)
            #location,_,error = blobLocator(self.currentImg)
            #location,_,error = houghCircles(self.currentImg)

            #Align with and kick ball
            if not error:
                #determining if ball is within alignment threshold
                window_y, window_x,_ = self.img.shape
                diff = location[0] - (window_x/2)

                if math.fabs(diff) < 20:
                    #move forward if within 20 pixels of center line
                    self.cmd.linear.x = 1 
                    self.cmd.angular.z = 0
                else:
                    #max diff is half window size (~320)
                    kp = .003  
                    gauss = (-300,0.1)
                    #setting twist control
                    self.cmd.linear.x = gauss[1]*math.exp(diff**2/gauss[0]**2)                      
                    self.cmd.angular.z = -diff*kp

        # change state if needed
        if error:
            print("The ball is a lie")
            self.state = self.determineBall
            self.transition = True           

    def run(self):
        """ The main run loop"""
        rospy.on_shutdown(lambda: self.cmd_pub.publish(Twist()))

        while not rospy.is_shutdown():
            if self.img is not None:
                #show image
                cv2.imshow('video_window', self.img)
                cv2.waitKey(5) 

                #perform current state actions
                self.state()
            else:
                self.cmd = Twist()

            #move robot
            self.cmd_pub.publish(self.cmd)
            self.rate.sleep()

if __name__ == '__main__':
    node = NeatoSoccerPlayer("/camera/image_raw")
    node.run()
