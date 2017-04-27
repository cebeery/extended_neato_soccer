#!/usr/bin/env python

"""
balldetector.py
---------------

An interface to the neural network that allows the classification of images as
containing or not containing a ball.

See example usage at bottom of file.

"""

from neuralnet import Network
import os


class BallDetector(object):
	def __init__(self):
		# TODO: Load NN weights & configure
		pass

	def classify(self, img):
		return True

if __name__ == "__main__":
	# bd = BallDetector()				# Create Ball Detector class
	# print bd.classify("foo")	# Classify an image as ball-containing or not

	# TODO: Cleanup this code into something reusable
	imageFolder = '../../images/training-imgs'

	# Train the network with all of the images in the example folder,
	# using their filenames ("no_boxes" or "boxes" as the tag)
	n = Network()
	n.trainWith(os.path.join(imageFolder), presentPrefix="ball")

	# NOTE: the predictions below are NOT good form - we test on the images
	# that we trained on. This is an example, not suitable for production

	testFP = os.path.join(imageFolder, "test")
	# Predict how likely it is that each of the box images contains a box.
	# Ideally 1 for all of the images in this category.
	posImgs = [
		os.path.join(testFP, e) for e in os.listdir(testFP)
		if os.path.isfile(os.path.join(testFP, e)) and e[:2] == 'ba'
	]

	print "Predictions for ball images:\n{}".format(
		n.predict(posImgs)
	)

	negImgs = [
		os.path.join(testFP, e) for e in os.listdir(testFP)
		if os.path.isfile(os.path.join(testFP, e)) and e[:2] == 'no'
	]

	# Predict how likely it is that each of the (first 10) non-box images
	# contains a box. Ideally 0 for all of the images in this category.
	print "Predictions for non-ball images:\n{}".format(
		n.predict(negImgs)
	)