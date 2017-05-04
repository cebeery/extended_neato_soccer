#!/usr/bin/env python

"""
balldetector.py
---------------

An interface to the neural network that allows the classification of images as
containing or not containing a ball.

Running this file will retrain the underlying network. Simply loading the
BallDetector class is the general client use-case.

"""

# Library imports
import os

# Program imports
from config import CONFIG
import neuralnet
import utils


# Load Constants
NETWORK_FILENAME = CONFIG.get("NETWORK_FILENAME")
THRESHOLD = CONFIG.get("THRESHOLD")
BALL_TAG = CONFIG.get("BALL_TAG")
NO_BALL_TAG = CONFIG.get("NO_BALL_TAG")
NN_IMAGE_SIZE = CONFIG.get("NN_IMAGE_SIZE")
PROJECT_DIR = CONFIG.get("PROJECT_DIR")
COLOR = CONFIG.get("COLOR")


# Define Custom Constants
NETWORK_FILEPATH = os.path.join(PROJECT_DIR, 'scripts/NeuralNet/save')

class BallDetector(object):
	"""
	A convenient adapter interface for a particular pre-trained neural-net.
	Pre-configured to load the network and return a simple boolean response
	when presented with an image piped in from ROS.
	"""

	def __init__(
		self,
		networkFilename=NETWORK_FILENAME,
		networkFilepath=NETWORK_FILEPATH,
		threshold=THRESHOLD,
		nnImageSize=NN_IMAGE_SIZE
	):
		"""
		Initializes the ball detector class.

		Input:
			networkFilename (string): the filename of the underlying network to load.
			threshold (float): the threshold of confidence above which an image is
				considered to contain a ball.
			nnImageSize (int): the expected pixel-size of the downsampled image.

		Outputs:
			None

		"""

		self.network = neuralnet.load(networkFilename, networkFilepath)
		self.threshold = threshold
		self.nnImageSize = nnImageSize

	def classify(self, img):
		"""
		Classifies a passed image, as bridged from ROS to OpenCV as either True,
		if it contains a ball, or False otherwise.

		Input:
			img (numpy.ndarray): the image to classify

		Output:
			(boolean): True, if it contains a ball. False otherwise.

		"""
		formattedImg = utils.formatImage(img, self.nnImageSize, color=COLOR)
		formattedImg = formattedImg.reshape([3, 64, 64])
		prediction = self.network.predict(formattedImg)[0]
		print prediction
		return prediction > self.threshold


# Train neural net
if __name__ == "__main__":
	# Set up labels for classifying data
	labels = {
		BALL_TAG: True,
		NO_BALL_TAG: False,
	}

	# Construct path to training images
	imgDir = os.path.join(PROJECT_DIR, 'images/neural-net/{}'.format(NN_IMAGE_SIZE))

	# Train network
	n = neuralnet.Network()
	print "Average error in test set: {}".format(n.trainTest(labels, imgDir))
	n.save(NETWORK_FILENAME, NETWORK_FILEPATH)