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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Constants
IMAGE_SIZE = 128
BALL_TAG = "ball"
NO_BALL_TAG = "no_ball"

CLASSIFICATION_CONTROLS = {
  BALL_TAG: pygame.K_RIGHT,
  NO_BALL_TAG: pygame.K_LEFT,
}

SAVE_PATH = os.path.join(
  r.get_path('extended_neato_soccer'),
  'images/training-imgs'
)


def filenumber(filename, tag):
  return int(filename[-7:-4]) if filename[:len(tag)] == tag else -1


class BallTinder(object):
  def __init__(
    self,
    filepath=SAVE_PATH
  ):
    rospy.init_node('balltinder')
    rospy.Subscriber('camera/image_raw', Image, bt.tag)
    self.pub = rospy.Publisher('camera/processed', Image, queue_size=10)

    # Use pygame to capture key input and show the images as they are processed
    pygame.init()
    pygame.display.set_mode((IMAGE_SIZE, IMAGE_SIZE))

    self.bridge = CvBridge()
    self.filepath = filepath

    # Read the directory of current images and start numbering
    # for new images such that they will continue the sequence
    imgNames = [f for f in os.listdir(self.filepath) if os.path.isfile(f)]
    self.noBallCount = max(imgNames, lambda fn: filenumber(fn, NO_BALL_TAG))
    self.ballCount = max(imgNames, lambda fn: filenumber(fn, BALL_TAG))

    # Print instructions
    print "Welcome to ball tinder, a helper for rapidly classifying images as " \
      "containing or not containing balls. In the window that is displayed, you " \
      "will see an image from the neato's camera feed. For as long as the image " \
      "contains a soccer ball, hold the right arrow key. For any times the image " \
      "does not contain a soccer ball, hold the left arrow key. If the ball is " \
      "partially on screen, the image contains a ball. If you are unsure, you can " \
      "opt to not press either arrow key."

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      r.sleep()

  def tag(self, imageMsg):
    # Format the image appropriately
    rawImg = self.bridge.imgmsg_to_cv2(imageMsg, desired_encoding="mono8")
    formattedImg = utils.formatImage(rawImg)

    # Display & publish current image
    self.pub.publish(self.bridge.cv2_to_imgmsg(formattedImg))
    # TODO: pygame display code

    # Classify and save image file appropriately based on any current key-press
    keyInputs = pygame.key.get_pressed()
    if keyInputs[CLASSIFICATION_CONTROLS[BALL_TAG]]:
      recordImg(True, formattedImg)
    elif keyInputs[CLASSIFICATION_CONTROLS[NO_BALL_TAG]]:
      recordImg(False, formattedImg)

  def recordImg(self, isBall, formatImg):
    if isBall:
      count = self.ballCount
      tag = BALL_TAG
      self.ballCount +=1
    else:
      count = self.noBallCount
      tag = NO_BALL_TAG
      self.noBallCount += 1

    filename = "{}_{}.png".format(tag, str(count.zfill(3)))
    cv2.imwrite(os.path.join(self.filepath, filename), formattedImg)


if __name__ == "__main__":
  bt = BallTinder(path)