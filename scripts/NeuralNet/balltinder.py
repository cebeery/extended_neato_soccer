#!/usr/bin/env python

"""
balltinder.py
---------------

A utility ros node for reading from the camera stream, down-sampling the images,
and labeling them in real time to quickly build a library of training images.

Example code at for this library is at the bottom of the file.

"""

import rospy
import rospkg
import pygame   # Odd, I know, but it has the best utilities for managing key presses
import utils
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from config import CONFIG

# Load Constants
ORIG_IMAGE_SIZE = 				CONFIG.get("ORIG_IMAGE_SIZE")
NN_IMAGE_SIZE = 					CONFIG.get("NN_IMAGE_SIZE")
BALL_TAG = 								CONFIG.get("BALL_TAG")
NO_BALL_TAG = 						CONFIG.get("NO_BALL_TAG")
SKIPS_TO_RECORD_RATIO = 	CONFIG.get("SKIPS_TO_RECORD_RATIO")


# Define Custom Constants
CLASSIFICATION_CONTROLS = {
	BALL_TAG: pygame.K_RIGHT,
	 NO_BALL_TAG: pygame.K_LEFT,
}

FEEDBACK_COLORS = {
	BALL_TAG: (0, 255, 0),
	NO_BALL_TAG: (255, 0, 0),
	None: None,
}


class UserInterface(object):
	"""
	A pygame wrapper that provides an GUI for Ball Tinder.
	"""

	def __init__(self, size, title="Display"):
		"""
		Initializes the UI.

		Input:
			size (int tuple): the pixel dimension of the GUI in (width, height) format
			title (string): the title of the GUI window

		Output:
			None

		"""

		pygame.init()
		pygame.display.set_caption(title)
		self.screen = pygame.display.set_mode((size, size))
		self.size = size

	def getTag(self):
		"""
		Gets the tag, if any, corresponding to the user's current keyboard inputs.
		Not guaranteed to work for multiple, simultaneous key-presses.

		Input:
			None

		Output:
			(string): a tag for use in accessing variable and classifying images

		"""

		pygame.event.pump()
		for tag, key in CLASSIFICATION_CONTROLS.items():
			if pygame.key.get_pressed()[key]:
				return tag

	def showImg(self, img, border=None):
		"""
		Displays an image in the GUI with an optional border.

		Input:
			img (numpy.ndarray): an image (expected to have 3 channels)
			border (int tuple): an RGB color tuple in format (R, G, B)

		Output:
			None

		"""

		# Show the image pygame display
		surfImg = pygame.surfarray.make_surface(img)
		self.screen.blit(pygame.transform.rotate(surfImg, -90), (0, 0))

		# Outline the window with a border of the passed color
		if border:
			pygame.draw.rect(
				self.screen,
				border,
				pygame.Rect(0, 0, self.size, self.size),
				10   # Width of border. 0 means fill
			)

		# Repaint the display
		pygame.display.flip()


class BallTinder(object):
	"""
	A utility program for rapidly classifying images from a video stream
	as containing or not-containing a ball, then reformatting and saving
	those images for use in neural network training.
	"""

	def __init__(self, savePath):
		"""
		Initializes the program.

		Input:
			savepath (string): the filepath to which Ball Tinder should save images

		Output:
			None

		"""

		# Variables Setup
		self.ui = UserInterface(ORIG_IMAGE_SIZE)
		self.bridge = CvBridge()
		self.savePath = savePath
		self.skipCounter = 0

		# Get the highest number of the images with each tag in the save directory
		# so that numbering can be initialized at that count
		imgLists = utils.imgLists(savePath, labelNames=(BALL_TAG, NO_BALL_TAG))
		self.counts = {
			BALL_TAG: max([utils.filenumber(el) for el in imgLists[BALL_TAG]] or [0]),
			NO_BALL_TAG: max([utils.filenumber(el) for el in imgLists[NO_BALL_TAG]] or [0]),
		}

		# Start ROS - done last so that images are not received before UI is set up
		rospy.init_node('balltinder')
		self.pub = rospy.Publisher('camera/processed', Image, queue_size=10)
		rospy.Subscriber('camera/image_raw', Image, self.tag)

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

	def tag(self, imgMsg):
		"""
		Classifies an image message as containing or not containing a ball based on
		the current UI inputs. Displays image to UI, formats the image, republishes
		and saves the formatted image when appropriate.

		Input:
			imgMsg (sensor_msgs.msg.Image): a camera image, gray-scale or color

		Output:
			None

		"""

		tag = self.ui.getTag()

		# Format the image
		rawImg = self.bridge.imgmsg_to_cv2(imgMsg, desired_encoding="passthrough")
		formattedImg = utils.formatImage(rawImg, NN_IMAGE_SIZE)

		# Publish and display the image
		self.pub.publish(self.bridge.cv2_to_imgmsg(formattedImg))
		self.ui.showImg(utils.crop(rawImg), FEEDBACK_COLORS[tag])

		# Save and tag every _th image where the _ is set by the config
		self.skipCounter = (self.skipCounter + 1) % SKIPS_TO_RECORD_RATIO
		if tag and not self.skipCounter:
			self.saveImg(formattedImg, tag)

	def saveImg(self, img, tag):
		"""
		Saves an image using the provided tag for naming. For example, provided the
		tag "ball", this method might save the image as, eg. "ball_000245.png".

		Input:
			img (numpy.ndarray): the image to save
			tag (string): the tag with which to save the image

		Output:
			None

		"""

		filename = "{}_{}.png".format(tag, str(self.counts[tag]).zfill(6))
		cv2.imwrite(os.path.join(self.savePath, filename), img)
		self.counts[tag] += 1


# Example Code
if __name__ == "__main__":
	r = rospkg.RosPack()
	path = os.path.join(
		r.get_path('extended_neato_soccer'),
		'images/neural-net'
	)
	bt = BallTinder(path)