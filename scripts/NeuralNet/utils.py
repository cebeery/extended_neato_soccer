#!/usr/bin/env python

"""
utils.py
--------

Contains utility functions for use in other script

Example usage at bottom of file.

"""

import cv2
import os
import numpy as np
from config import CONFIG

# Constants
NN_IMAGE_SIZE = CONFIG.get("NN_IMAGE_SIZE")
BALL_TAG = CONFIG.get("BALL_TAG")
NO_BALL_TAG = CONFIG.get("NO_BALL_TAG")

def formatImageFiles(srcPath, dstPath, nnImgSize=NN_IMAGE_SIZE):
	"""
	Formats standard image files for use in neural net training.

	Loads all images from the source path, gray-scales them,
	crops them to a square aspect ratio, and down-samples them
	to nnImgSize x nnImgSize. Saves the formatted images to the
	destination path. Will overwrite if the source and
	destination paths are the same.

	Input:
		srcPath (string): the source path for the images
		dstPath (string): the destination path for the images
		nnImgSize (int, optional): the dimension of the output

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
			formatImage(raw, NN_IMAGE_SIZE)
		)


def crop(rawImg):
	"""
	Crops the given image to a square with edge length equal to the shorter
	dimension of the passed image. The cropped image will be a centered subset
	of the original.

	Input:
		rawImg (numpy.ndarray): the raw image

	Output:
		(numpy.ndarray): the cropped image, centered within the original

	"""

	# Crop image to be a square
	height, width = rawImg.shape[:2]
	boxSize = height if height < width else width

	return rawImg[
		(height - boxSize) / 2 : (height + boxSize) / 2,
		(width - boxSize) / 2  : (width + boxSize) / 2,
	]


def formatImage(rawImg, nnImgSize):
	"""
	Formats an image ndarray into the format needed for
	training the neural network.

	Namely, converts to a grayscaled, square-cropped,
	nnImgSize x nnImgSize image.s

	Input:
		rawImg	(numpy.ndarray): the raw image
		nnImgSize (int, optional): the dimension of the output

	Output:
		(numpy.ndarray): the formatted image

	"""

	# Convert image to gray-scale (if it is not already).
	# Accept either formats with 3 channels, eg. (640, 480, 3)
	# Or 1 channel, eg. (640, 480, 1) and reformat to
	# a more compact (640, 480) representation
	grayImg = cv2.cvtColor(rawImg, cv2.COLOR_BGR2GRAY)  \
		if len(rawImg.shape) > 2 and rawImg.shape[2] != 1 \
		else rawImg.reshape(rawImg.shape[:2])

	cropped = crop(grayImg)

	# Down-sample the image
	return cv2.resize(cropped, (nnImgSize, nnImgSize))


def filenumber(filename):
	"""
	Utility for extracting the file-number from images.

	Input:
		filename (string): the name of an image file, eg. "ball_000425.png"

	Output:
		(int): the number associated with the image eg, in the above, "425"

	"""

	return int(filename[-7:-4])


def hasTag(filename, tag):
	"""
	Utility to check if a certain filename is prefixed with a particular
	tag.

	Input:
		filename (string): the name of a file, eg. "ball_000425.png"
		tag (string): the prefix to test against the images, eg. "ball"

	Output:
	 (bool): True, if `filename` is preceded by `tag`. False otherwise.

	"""
	return filename[:len(tag)] == tag

def listImgs(path=None):
	if not path:
		r = rospkg.RosPack()
		path = os.path.join(
			r.get_path('extended_neato_soccer'),
			'images/training-imgs',
		)

	files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]

	return {
		BALL_TAG: [f for f in files if hasTag(f, BALL_TAG)],
		NO_BALL_TAG: [f for f in files if hasTag(f, NO_BALL_TAG)]
	}


if __name__ == "__main__":

	# Convert all the files in the import directory to
	# the format required by the neural network and store
	# the results in the example directory
	formatImageFiles(
		'../../images/training-imgs/import',
		'../../images/training-imgs/example',
	)