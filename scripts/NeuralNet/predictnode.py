#!/usr/bin/env python

"""
predictnode.py
---------------

A ros node for testing out the neural network's live prediction
capability.

When executed, this script launches a ros node and begins to
print a classification as well as a confidence value, both
corresponding to whether or not there is a ball in the camera
frame.

"""


from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from balldetector import BallDetector
import cv2
import rospy



class TestNode(object):
	def __init__(self):
		self.bd = BallDetector()
		self.bridge = CvBridge()
		cv2.namedWindow('video_window')
		self.img = None


		# Setup ROS Node
		rospy.init_node('ball_detection_test')
		rospy.Subscriber("/camera/image_raw", Image, self.isBall)

		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.img is not None:
				cv2.imshow('video_window', self.img)
				cv2.waitKey(5)
			r.sleep()

	def isBall(self, imgMsg):
		self.img = self.bridge.imgmsg_to_cv2(imgMsg, desired_encoding="bgr8")
		print self.bd.classify(self.img)

TestNode()