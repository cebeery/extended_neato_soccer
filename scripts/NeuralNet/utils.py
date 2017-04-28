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


def imgLists(filepath, labelNames=None):
	"""
	Returns all the files in the directory given by 'filepath' and organizes them
	into a dictionary keyed by label if labelNames are provided.

	Input:
		filepath (string): the directory to search for files
		labelNames (string list): a list of possible labelNames to apply,
			eg. ("ball", "no_ball")

	Output:
		(str->(str list) dict): a mapping from each label to the list of files with
			that label

	"""

	files = [
		f for f in os.listdir(filepath)
		if os.path.isfile(os.path.join(filepath, f))
	]

	return {
		label: [f for f in files if hasTag(f, label)]
		for label in labelNames
	} if labelNames else files


def getLabel(filename, labelNames):
	"""
	Given a list of possible labelNames for a filename, match the filename to a label.

	Input:
		filename (string): the filename to match to a label, eg. "ball_000567.png"
		labelNames (string list): a list of possible labelNames to apply,
			eg. ("ball", "no_ball")

	Output:
		(string): the label that corresponds to the filename

	"""

	for labelName in labelNames:
		if labelName == filename[:len(labelName)]:
			return labelName


def labelsFor(labels, filenames):
	"""
	Return a label list that matches the labels of the passed array of filenames.

	Input:
		labels (str->int dict): the desired labels for each tag, eg: {'ball': 1}
		filenames (string list): the filenames to be labeled

	Output:
		(int list): the labels corresponding by index to the filenames

	"""

	return [
		labels[getLabel(filename, labels.keys())]
		for filename in filenames
	]


# Example Code
if __name__ == "__main__":

	# Convert all the files in the import directory to
	# the format required by the neural network and store
	# the results in the example directory
	formatImageFiles(
		'../../images/neural-net/import',
		'../../images/neural-net/example',
	)