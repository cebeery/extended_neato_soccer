#!/usr/bin/env python
import rospy, math, cv2, os, time
import numpy as np
import automaton as a 
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import  Image
from neato_node.msg import Bump



class NeatoSoccerPlayer(a.Automaton):

    def __init__(self, 
            name="neato_soccer",
            states={"DetermineBall":DetermineBall(), "LocateBall":LocateBall(), "Align":Align(), "Kick":Kick()},
            initialState="DetermineBall",
            debug=False
        ):

        super(NeatoSoccerPlayer, self).__init__(name, states, initialState, debug)

        #initialize vision attributes
        self.img = None         	#raw image from sensor callback
        self.currentImg = None  	#current image being processed 
        self.bridge = CvBridge() 
        cv2.namedWindow('video_window')

    def processImage(self, msg):
        """image callback that stores image"""
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="brg8")

    def processBump(self, msg):
        """Signals node shutdown on front bumper trigger"""
        if msg.leftFront or msg.rightFront:
	    rospy.signal_shutdown("Kick detected; Node ended")

if __name__ == "__main__":
   player = NeatoSoccerPlayer()  
   player.main()

