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

def formatImages(srcPath, dstPath):
	"""
	Formats standard images for use in neural net training.

	Loads all images from the source path, gray-scales them,
	crops them to a square aspect ratio, and down-samples them
	to 128x128. Saves the formatted images to the destination
	path. Will overwrite if the source and destination paths
	are the same.

	Input:
		srcPath (string): the source path for the images
		dstPath (string): the destination path for the images

	Output:
		None

	"""

	imgNames = os.listdir(srcPath)

	for imgName in imgNames:
		raw = cv2.imread(
			os.path.join(srcPath, imgName),
			cv2.IMREAD_GRAYSCALE
		)

		# Crop image to be a square
		height, width = raw.shape
		boxSize = height if height < width else width

		cropped = raw[
			(height - boxSize) / 2 : height + boxSize,
			(width - boxSize) / 2  : width + boxSize,
		]

		# Downsample the image
		downsampled = cv2.resize(cropped, (128, 128))

		cv2.imwrite(
			os.path.join(dstPath, imgName),
			downsampled
		)

if __name__ == "__main__":

	# Convert all the files in the import directory to
	# the format required by the neural network and store
	# the results in the example directory
	formatImages(
		'./training-imgs/import',
		'./training-imgs/example',
	)