#!/usr/bin/env python

"""
neuralnet.py
----------

Sets up a neural network configuration for training.

See example usage at bottom of file.

"""

from lasagne import layers
from lasagne.updates import nesterov_momentum
from nolearn.lasagne import NeuralNet
import os
import rospkg
import numpy as np
import Image
from config import CONFIG

# Load Constants
NN_IMAGE_SIZE = CONFIG.get("NN_IMAGE_SIZE")
BALL_TAG = CONFIG.get("BALL_TAG")
NO_BALL_TAG = CONFIG.get("NO_BALL_TAG")

# Custom Constants
LABELS = {
	BALL_TAG: 1,
	NO_BALL_TAG: 0,
}

class Network(NeuralNet):
	def __init__(self):

		# Configure neural net
		super(Network, self).__init__(
			# Topography of network
			layers=[
				('input', layers.InputLayer),
				('hidden1', layers.DenseLayer),
				('hidden2', layers.DenseLayer),
				('hidden3', layers.DenseLayer),
				('hidden4', layers.DenseLayer),
				('hidden5', layers.DenseLayer),
				('output', layers.DenseLayer),
			],

			# Layer parameters
			input_shape=(None, NN_IMAGE_SIZE, NN_IMAGE_SIZE),		# Shape of the input (# imgs, rows, cols)
			hidden1_num_units=100,						# Number of units in each hidden layer
			hidden2_num_units=100,						# Number of units in each hidden layer
			hidden3_num_units=100,						# Number of units in each hidden layer
			hidden4_num_units=100,						# Number of units in each hidden layer
			hidden5_num_units=100,						# Number of units in each hidden layer
			output_nonlinearity=None,				# BLACKBOX: Output uses identity function
			output_num_units=1,							# Target values (Ball: y/n, x-coord, y-coord, radius)

			# Optimization method
			update=nesterov_momentum,				# BLACKBOX
			update_learning_rate=0.01,			# BLACKBOX
			update_momentum=0.9,						# BLACKBOX

			regression=True,								# BLACKBOX
			max_epochs=1000,								# BLACKBOX
			verbose=1,											# Print output trace or not during training
		)

	def trainWith(self, labels=LABELS, imgsPath=):
		# Get a list of all of the image paths from the imgPath directory
		imgNames = [
			e for e in os.listdir(imgsPath)
			if os.path.isfile(os.path.join(imgsPath, e))
		]
		imgPaths = [
			os.path.join(imgsPath, name)
			for name in imgNames
		]

		# Format images into the format expected by lasagne
		# (batch size, rows, columns) and normalize
		# pixel values into the range [0-1]
		nnData = np.array([
			np.array(Image.open(imgPath))
			for imgPath in imgPaths
		]) / 255.

		nnLabels = np.array([
			True if imgName[:len(presentPrefix)] == presentPrefix else False
			for imgName in imgNames
		])

		print "\nFitting model to {} images.\n".format(nnData.shape[0])

		self.fit(nnData, nnLabels)

	def predict(self, imgPaths):
		img = np.array([
			np.array(Image.open(imgPath))
			for imgPath in imgPaths
		]) / 255.

		return super(Network, self).predict(img)

if __name__ == "__main__":
	r = rospkg.RosPack()
	imgDir = os.path.join(
		r.get_path('extended_neato_soccer'),
		'images/training-imgs',
	)

	imageFolder = 

	# Train the network with all of the images in the example folder,
	# using their filenames ("no_boxes" or "boxes" as the tag)
	n = Network()
	n.trainWith(os.path.join(imageFolder, 'example/train'), presentPrefix="boxes")

	# NOTE: the predictions below are NOT good form - we test on the images
	# that we trained on. This is an example, not suitable for production

	# Predict how likely it is that each of the box images contains a box.
	# Ideally 1 for all of the images in this category.
	print "Predictions for box images:\n{}".format(
		n.predict([
			os.path.join(imageFolder, 'example/test/boxes_{}.png'.format(str(i).zfill(3)))
			for i in range(5, 6)
		])
	)

	# Predict how likely it is that each of the (first 10) non-box images
	# contains a box. Ideally 0 for all of the images in this category.
	print "Predictions for non-box images:\n{}".format(
		n.predict([
			os.path.join(imageFolder, 'example/test/no_boxes_{}.png'.format(str(i).zfill(3)))
			for i in range(11, 14)
		])
	)