#!/usr/bin/env python

"""
utils.py
--------

Contains utility functions for use in other parts of the
neural network program.

Example usage at bottom of file.

"""

import cv2
import os
import numpy as np

# Constants
IMAGE_SIZE = 128


def formatImageFiles(srcPath, dstPath, imgSize=IMAGE_SIZE):
	"""
	Formats standard image files for use in neural net training.

	Loads all images from the source path, gray-scales them,
	crops them to a square aspect ratio, and down-samples them
	to imgSize x imgSize. Saves the formatted images to the
	destination path. Will overwrite if the source and
	destination paths are the same.

	Input:
		srcPath (string): the source path for the images
		dstPath (string): the destination path for the images
		imgSize (int, optional): the dimension of the output

	Output:
		None

	"""

	imgNames = os.listdir(srcPath)

	for imgName in imgNames:
		raw = cv2.imread(
			os.path.join(srcPath, imgName),
			cv2.IMREAD_GRAYSCALE
		)

		cv2.imwrite(
			os.path.join(dstPath, imgName),
			formatImage(raw)
		)


def formatImage(rawImg, imgSize=IMAGE_SIZE):
	"""
	Formats an image ndarray into the format needed for
	training the neural network.

	Namely, converts to a grayscaled, square-cropped,
	imgSize x imgSize image.s

	Input:
		rawImg	(numpy.ndarray): the raw image
		imgSize (int, optional): the dimension of the output

	Output:
		(numpy.ndarray): the formatted image

	"""

	# Convert image to gray-scale (if it is not already)
	grayImg = cv2.cvtColor(rawImg, cv2.COLOR_BGR2GRAY) \
	if len(rawImg.shape) > 2 else rawImg

	# Crop image to be a square
	height, width = grayImg.shape
	boxSize = height if height < width else width

	cropped = grayImg[
		(height - boxSize) / 2 : height + boxSize,
		(width - boxSize) / 2  : width + boxSize,
	]

	# Down-sample the image
	return cv2.resize(cropped, (imgSize, imgSize))


if __name__ == "__main__":

	# Convert all the files in the import directory to
	# the format required by the neural network and store
	# the results in the example directory
	formatImageFiles(
		'../../images/training-imgs/import',
		'../../images/training-imgs/example',
	)