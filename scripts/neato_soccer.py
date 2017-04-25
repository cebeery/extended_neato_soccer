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
from VisionSuite.blobLocator import blobLocator
from VisionSuite.colorFilteredCOM import colorFilteredCOM


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
        #states = determineBall, alignNeato, kickBall]
        self.state = self.determineBall
        self.transition = True

        # Setup Neural Net Class Instance
        self.nn = BallDetector()

        # Setup raw camera visualization
        self.img = None                 #the latest image from the camera
        self.currentImg = None          #curretn image being processed
        self.bridge = CvBridge()        #used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window') #create window for live video stream

        # Setup ROS Node
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
        using a neural net
        At Start    > stop neato motion, set currentImg to img
        Transition 1: condition(ball found) > alignNeato state
        Transition 2: condition(no ball) > end node
        """

        # init change state flag
        ball = None

        # perform state funtionally
        if self.transition:
            print "Transitioning to determineBall state"
            self.cmd = Twist()
            self.currentImg = self.img
            self.transition = False
        else:
            print "Looking for ball"
            ball = self.nn.classify(self.currentImg)

        # change state if needed
        if ball:
            print "Ball exists"
            self.state = self.alignNeato
            self.transition = True
        elif ball is not None:
            print "Where ball? Want ball!"
            rospy.signal_shutdown("No Soccerball was detected in view")     

    def alignNeato(self):
        """
        Determines the location of the ball with in the image being processed
        and rotates neato to line up with ball 
        At Start    > stop neato motion
        Transition 1: condition(ball centered) > kickBall state
        """

        # init change state flag
        aligned = False

        # perform state funtionally
        if self.transition:
            print "Transitioning to alignNeato state"
            self.cmd = Twist()
            self.transition = False

        else:
            #use on of vision suite methods
            location,_ = colorFilteredCOM(self.currentImg)
            #location,_ = blobLocator(self.currentImg)

            #determining if ball is within alignment threshold
            window_y, window_x,_ = self.currentImg.shape
            diff = location[0] - (window_x/2)

            if math.fabs(diff) < 20:
                #move forward if within 20 pixels ahead
                aligned = True
            else:
                #setting proportional twist control
                kp = .005           
                self.cmd.angular.z = -diff*kp

        # change state if needed
        if aligned:
            print "Neato aligned with ball"
            self.state = self.kickBall
            self.transition = True

    def kickBall(self):
        """
        Drive forward to kick ball until node ended
        At Start    > stop neato motion
        """

        # perform state funtionally
        if self.transition:
            print "Transitioning to kickBall state"
            self.cmd = Twist()
            self.transition = False
        else:
            self.cmd.linear.x = 0.3 
            self.transition = True
            #possibly temp -> stop on circle too big instead of rechecking/remove cmd 0 in transitions
            self.state=self.determineBall

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
