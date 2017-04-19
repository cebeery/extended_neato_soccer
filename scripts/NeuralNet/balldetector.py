#!/usr/bin/env python

"""
balldetector.py
---------------

An interface to the neural network that allows the classification of images as
containing or not containing a ball.

See example usage at bottom of file.

"""


class BallDetector(object):
	def __init__(self):
		# TODO: Load NN weights & configure
		pass

	def classify(self, img):
		return True

if __name__ == "__main__":
	bd = BallDetector()				# Create Ball Detector class
	print bd.classify("foo")	# Classify an image as ball-containing or not