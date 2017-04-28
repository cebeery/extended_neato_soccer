#!/usr/bin/env python

"""
neuralnet.py
----------

Sets up a neural network configuration for training.

See example usage at bottom of file.

"""

import os
import rospkg
import utils
import numpy as np
import Image
from config import CONFIG
import lasagne
import cPickle as pickle
from lasagne import layers
from lasagne.updates import nesterov_momentum
from nolearn.lasagne import NeuralNet
import random


# Load Constants
NN_IMAGE_SIZE = CONFIG.get("NN_IMAGE_SIZE")


def load(filename="neural-net.pkl", filepath="save"):
	"""
	Load a neural network, likely trained, from its save file.

	Input:
		filename (string): the name of the save-file to open. Default: "neural-net.pkl"
		filepath (string): the path to the save-file. Default: "save"

	Output:
		(neuralnet.Network): a neural network

	"""
	with open(os.path.join(filepath, filename), 'r') as f:
		network = pickle.load(f)

	return network


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
			input_shape=(None, 32, 32),		# 32x32 input pixels per batch
			hidden1_num_units=100,				# Number of units in hidden layer
			hidden2_num_units=100,				# Number of units in hidden layer
			hidden3_num_units=100,				# Number of units in hidden layer
			hidden4_num_units=100,				# Number of units in hidden layer
			hidden5_num_units=100,				# Number of units in hidden layer
			output_nonlinearity=None,			# Output layer uses identity function
			output_num_units=1,						# Target values (Ball: y/n, x-coord, y-coord, radius)

			# Optimization method
			update=nesterov_momentum,			# a gradient descent algorithm
			update_learning_rate=0.01,		# BLACKBOX
			update_momentum=0.9,					# BLACKBOX

			regression=True,							# BLACKBOX
			max_epochs=1500,							# Number of times to run and re-balance the network
			verbose=1,										# Print output trace or not during training
		)

	def trainTest(self, labels, filepath, testRatio=0.2):
		imgNames = utils.imgLists(path=filepath)
		testSet = random.sample(imgNames, int(testRatio * len(imgNames)))
		trainSet = [imgName for imgName in imgNames if imgName not in testSet]

		self.trainWith(labels, trainSet, filepath)

		testLabels = utils.labelsFor(labels, testSet)
		predictions = self.predictFiles(testSet, filepath)

		# Calculate and return the average error over the test set
		errors = [
			abs(testLabels[i]-predictions[i])
			for i in xrange(len(testLabels))
		]

		avgError = sum(errors)/len(errors)
		return avgError[0]


	def trainWith(self, labels, filenames, filepath):
		"""
		Trains the model with the files at file-path based on the passed
		label mapping.
		"""

		imgPaths = [os.path.join(filepath, name) for name in filenames]

		# Format images into the format expected by lasagne (batch size, rows,
		# columns) and normalize pixel values into the range [0-1]
		nnData = np.array([
			np.array(Image.open(imgPath))
			for imgPath in imgPaths
		]) / 255.

		# Match each image name with a label's value to generate an array
		# classifying each image appropriately
		nnLabels = np.array(utils.labelsFor(labels, filenames))

		print "\nFitting model to {} images.\n".format(nnData.shape[0])

		self.fit(nnData, nnLabels)

	def predict(self, imgs):
		return super(Network, self).predict(imgs / 255.)

	def predictFiles(self, filenames, filepath):
		imgs = np.array([
			np.array(Image.open(os.path.join(filepath, filename)))
			for filename in filenames
		])

		return self.predict(imgs)

	def save(self, filename="neural-net.pkl", filepath="save"):
		with open(os.path.join(filepath, filename), 'w+') as f:
			pickle.dump(self, f)

if __name__ == "__main__":
	# Define labels
	BOX_LABEL = "boxes"
	NO_BOX_LABEL = "no_boxes"

	# Get a path to the example image train/test directory
	r = rospkg.RosPack()
	imgDir = os.path.join(
		r.get_path('extended_neato_soccer'),
		'images/neural-net/example'
	)

	# Train a network on the example images folder and save the network
	n = Network()
	avgError = n.trainTest({BOX_LABEL: 1, NO_BOX_LABEL: 0}, imgDir)
	print "Average error in test set predictions: {}".format(avgError)
	n.save('example-nn.pkl')

	# Load a saved neural network as 'n2'
	n2 = load('example-nn.pkl')

	# Use the loaded 'n2' network to predict the example images
	imgLists = utils.imgLists(path=imgDir, labels=(BOX_LABEL, NO_BOX_LABEL))
	print "Predictions for box images:\n{}".format(n2.predictFiles(imgLists[BOX_LABEL], imgDir))
	print "Predictions for non-box images:\n{}".format(n2.predictFiles(imgLists[NO_BOX_LABEL], imgDir))