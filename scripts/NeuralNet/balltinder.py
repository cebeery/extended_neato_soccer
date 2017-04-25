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
IMAGE_SIZE = 480
DOWNSAMPLE_SIZE = 32
BALL_TAG = "ball"
NO_BALL_TAG = "no_ball"

# Because most images are very similar to the last, this establishes a number
# of frames to drop for every frame that is saved in order to reduce the memory
# used on mostly repeated data of limited training utility.
SKIPS_PER_CAPTURE = 14

CLASSIFICATION_CONTROLS = {
  BALL_TAG: pygame.K_RIGHT,
  NO_BALL_TAG: pygame.K_LEFT,
}


def filenumber(filename):
  """
  Utility for extracting the file-number from images prefixed
  with a particular tag.

  Input:
    filename (string): the name of an image file, eg. "ball_000425.png"
    tag (string): the prefix of the images to be counted, eg. "ball"

  Output:
    (int): the number associated with the image if it matches the tag.
      -1 otherwise.

  """

  return int(filename[-7:-4])

def filterByTag(filename, tag):
  return filename[:len(tag)] == tag


class BallTinder(object):
  """
  A utility program for rapidly classifying images from a video stream
  as containing or not-containing a ball, then reformatting and saving
  those images for use in neural network training.
  """

  def __init__(
    self,
    savePath
  ):
    """
    The init method of BallTinder.

    Input:
      savePath (string): the directory to which to save recorded images

    Output:
      None

    """

    # Use pygame to capture key input and show the images as they are processed
    pygame.init()
    pygame.display.set_caption("Ball Tinder")   # Set the window title
    self.screen = pygame.display.set_mode((IMAGE_SIZE, IMAGE_SIZE))

    self.bridge = CvBridge()
    self.savePath = savePath
    self.skipCounter = 0

    # Read the directory of current images and start numbering
    # for new images such that they will continue the sequence
    imgNames = [f for f in os.listdir(self.savePath) if os.path.isfile(os.path.join(savePath, f))]
    self.noBallCount = max([filenumber(f) for f in imgNames if filterByTag(f, NO_BALL_TAG)] or [0])
    self.ballCount = max([filenumber(f) for f in imgNames if filterByTag(f, NO_BALL_TAG)] or [0])

    # Print instructions
    print "Welcome to ball tinder, a helper for rapidly classifying images as " \
      "containing or not containing balls. In the window that is displayed, you " \
      "will see an image from the neato's camera feed. For as long as the image " \
      "contains a soccer ball, hold the right arrow key. For any times the image " \
      "does not contain a soccer ball, hold the left arrow key. If the ball is " \
      "partially on screen, the image contains a ball. If you are unsure, you can " \
      "opt to not press either arrow key."

    rospy.init_node('balltinder')
    rospy.Subscriber('camera/image_raw', Image, self.tag)
    self.pub = rospy.Publisher('camera/processed', Image, queue_size=10)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      r.sleep()

  def tag(self, imageMsg):
    """
    Formats, republishes, tags, and saves an incoming image according to the
    key that the user is pressing at the time.

    Input:
      imageMsg (sensor_msgs.msg.Image): a camera image, gray-scale or color

    Output:
      None

    """

    # Format the image appropriately
    rawImg = self.bridge.imgmsg_to_cv2(imageMsg, desired_encoding="passthrough")
    croppedImg = utils.crop(rawImg)
    formattedImg = utils.formatImage(rawImg, imgSize=DOWNSAMPLE_SIZE)

    # Display & publish current image
    self.pub.publish(self.bridge.cv2_to_imgmsg(formattedImg))

    surfImg = pygame.surfarray.make_surface(croppedImg)
    self.screen.blit(pygame.transform.rotate(surfImg, -90), (0, 0))

    # Classify and save image file appropriately based on any current key-press
    if pygame.key.get_pressed()[CLASSIFICATION_CONTROLS[BALL_TAG]]:
      self.recordImg(True, formattedImg)
    elif pygame.key.get_pressed()[CLASSIFICATION_CONTROLS[NO_BALL_TAG]]:
      self.recordImg(False, formattedImg)

    # After all images for this iteration have been rendered into the buffer,
    # switch the buffer in to be the current display. IE. show everything
    # we've drawn since we last called this function
    pygame.display.flip()
    pygame.event.pump()

  def recordImg(self, isBall, formattedImg, drop=False):
    """
    Tags, and saves an incoming image according to its passed parameters.
    Additionally colors the user's viewport edges to indicate when an image
    is being classified as ball or non-ball in order to give visual feedback
    to the user's actions.

    Note: only records a fraction of the incoming images. Done in order to minimize
    storage space on the assumption that most directly sequential images will be
    very similar, so it would be wasteful to record all of them.

    Input:
      imageMsg (sensor_msgs.msg.Image): a camera image, gray-scale or color

    Output:
      None

    """

    if isBall:
      count = self.ballCount
      tag = BALL_TAG
      outlineColor = (0, 255, 0)
    else:
      count = self.noBallCount
      tag = NO_BALL_TAG
      outlineColor = (255, 0, 0)

    # Outline the window with a border in a color reflecting whether the
    # image has been coded as containing or not containing a ball.
    pygame.draw.rect(
      self.screen,
      outlineColor,
      pygame.Rect(0, 0, IMAGE_SIZE, IMAGE_SIZE),
      10   # Width of border. 0 means fill
    )

    if self.skipCounter != 0:
      self.skipCounter -= 1
      return
    else:
      self.skipCounter = SKIPS_PER_CAPTURE

    if isBall:
        self.ballCount += 1
    else:
      self.noBallCount += 1

    # Save the image with the classification encoded
    filename = "{}_{}.png".format(tag, str(count).zfill(6))
    cv2.imwrite(os.path.join(self.savePath, filename), formattedImg)


if __name__ == "__main__":
  r = rospkg.RosPack()
  path = os.path.join(
    r.get_path('extended_neato_soccer'),
    'images/training-imgs'
  )
  bt = BallTinder(path)