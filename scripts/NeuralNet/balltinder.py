#!/usr/bin/env python

"""
balltinder.py
---------------

A utility ros node for reading from the camera stream, down-sampling the images,
and labeling them in real time to quickly build a library of training images.

Example code at for this library is at the bottom of the file and will execute
if this file is run.

"""

import rospy
import rospkg
import pygame   # Odd, I know, but it has the best utilities for managing key presses
import utils
import cv2
import os
import sensor_msgs.msg import  Image
from cv_bridge import CvBridge

class BallTinder(object):
  def __init__(
    self,
    filepath=os.path.join(rospkg.get_path('extended_neato_soccer'), 'images/training-imgs')
  ):
    self.bridge = CvBridge
    self.filepath = filepath

    # Read the directory of current images and start numbering
    # for new images such that they will continue the sequence
    imgNames = os.listdir(self.filepath)
    self.noBallCount = 0
    self.ballCount = 0

    for imgName in imgNames:
      prefix = imgName[:2]
      number = int(imgName[-7:-4])
      if prefix == "no" and number > noBallCount:
        self.noBallCount = number
      elif number > ballCount:
        self.ballCount = number

  def tag(imageMsg):
    # Format the image appropriately
    rawImg = self.bridge.imgmsg_to_cv2(imageMsg, desired_encoding="mono8")
    formattedImg = utils.formatImg(rawImg)

    # Name image file appropriately based on current keypress
    ballInImage = pygame.key.get_pressed()[pygame.K_RIGHT]
    if ballInImage:
      self.ballCount += 1
  	  filename = "ball_{}.png".format(self.ballCount.zfill(3))
    else:
      self.noBallCount += 1
      filename = "no_ball_{}.png".format(self.noballCount.zfill(3))

    cv2.imwrite(os.path.join(self.filepath, filename), formattedImg)


def run(key=None):
  rospy.init_node('balltinder')
  bt = BallTinder()
  rospy.Subscriber('camera/raw', Image, bt.tag);

  # publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  print "Welcome to ball tinder. Hold the right arrow as the ball is in the frame, " \
  			"release if the ball has completely exited the frame."

  r = ropy.Rate(10)
  while not rospy.is_shutdown():
    rospy.spinOnce()
    r.sleep()

if __name__ == "__main__":
  run()