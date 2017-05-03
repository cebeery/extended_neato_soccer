#!/usr/bin/env python

"""
neuralnet.py
----------

Sets up a neural network configuration for training.

See example usage at bottom of file.

"""

# TODO: Add data-augmentation (transforms of existing images)
# TODO: Test convolution configuration

# Library Imports
import os
import Image
import lasagne
import random
import numpy as np
import cPickle as pickle
from lasagne import layers
from lasagne.updates import nesterov_momentum
from nolearn.lasagne import NeuralNet

# Project Imports
import utils
from config import CONFIG


# Load Constants
# NN_IMAGE_SIZE = CONFIG.get("NN_IMAGE_SIZE")
PROJECT_DIR = CONFIG.get("PROJECT_DIR")


def load(filename="neural-net.pkl", filepath="save"):
	"""
	Load a pickled neural network, including any configuration and training,
	from its save file. Networks can be saved with the corresponding
	'neuralnet.Network.save' method.

	Input:
		filename (string): the name of the save-file to open. Default: "neural-net.pkl"
		filepath (string): the path to the save-file. Default: "save"

	Output:
		(neuralnet.Network): a neural network

	"""
	n = Network()
	print "\n\n"
	print os.path.join(filepath, filename)
	print "\n\n"
	n.load_params_from(os.path.join(filepath, filename))
	return n


class Network(NeuralNet):
	"""
	They linchpin of the neural-net program, this is a configured wrapper over
	nolearn.lasagne's NeuralNet. Used to train, test, and predict images and
	configured particularly to have good accuracy for our data.
	"""

	def __init__(
		self
	):

		"""
		Initialize the Neural Net.

		Inputs:
			None.

		Outputs:
			None.

		"""

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
			input_shape=(None, 32, 32),		# Cannot include variables, unsure why
			hidden1_num_units=100,				# Number of units in hidden layer
			hidden2_num_units=100,				# Number of units in hidden layer
			hidden3_num_units=100,				# Number of units in hidden layer
			hidden4_num_units=100,				# Number of units in hidden layer
			hidden5_num_units=100,				# Number of units in hidden layer
			output_nonlinearity=None,			# Output layer uses identity function
			output_num_units=1,						# Target values

			# Optimization method
			update=nesterov_momentum,			# a gradient descent algorithm
			update_learning_rate=0.01,		# BLACKBOX
			update_momentum=0.9,					# BLACKBOX

			regression=True,							# BLACKBOX
			max_epochs=1500,							# Number of times to run and re-balance the network
			verbose=1,										# Print output trace or not during training
		)

	def trainTest(self, labels, filepath, testRatio=0.2):
		"""
		Perhaps the most convenient interface to network, trainTest splits off a
		fraction of the files in 'filepath' to test against after training the
		network. Returns the average error of the test set. This can then be used
		to validate model improvements. Note, however, that the test set is chosen
		in a non-deterministic manner, so average error return values may be
		statistical outliers.

		Input:
			labels (str->int dict): a mapping from possible file labels to the
				expected value of predicting those files, eg. {'ball': 1, 'no_ball': 0}
			filepath (string): the directory from which to load the train/test images
			testRatio (float): the fraction of the data to select as a test set

		Output:
			(float): the average error (predicted vs expected) of the test set

		"""

		imgNames = utils.imgLists(filepath)
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
		Trains the model to predict the labels given based on the data contained
		in the passed files.

		Input:
			labels (str->int dict): a mapping from possible file labels to the
				expected value of predicting those files, eg. {'ball': 1, 'no_ball': 0}
			filenames (string list): the files to use for training
			filepath (string): the directory from which to load the training files

		Output:
			None.

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
		"""
		An adapter for the superclass' predict function. Normalizes the incoming
		image (expected to range from 0-255) to a (0-1) range.

		Inputs:
			imgs (numpy.ndarray): a set of images with values in range 0-255

		Outputs:
			(numpy.ndarray): the predicted value of each inputted image.

		"""

		return super(Network, self).predict(imgs / 255.)

	def predictFiles(self, filenames, filepath):
		"""
		Predicts the values of the images included in filenames

		Input:
			filenames (string list): the names of the files to predict
			filepath (string list): the directory from which to read filenames

		Output:
			(numpy.ndarray): the predicted values of the inputted files

		"""

		imgs = np.array([
			np.array(Image.open(os.path.join(filepath, filename)))
			for filename in filenames
		])

		return self.predict(imgs)

	def save(self, filename="neural-net.pkl", filepath="save"):
		"""
		Saves this neural network (configuration and trained parameters) as a
		complete, standalone object to a file (a pickle file) that can be loaded
		with the matched 'neuralnet.load' function.

		Input:
			filename (string): the name of the file to which to save this neural-net
			filepath (string): the directory in which to save this neural-net

		Output:
			None.

		"""
		self.save_params_to(os.path.join(filepath, filename))


# Example Code
if __name__ == "__main__":

	# Define labels
	BOX_LABEL = "boxes"
	NO_BOX_LABEL = "no_boxes"

	# Get a path to the example image train/test directory
	imgDir = os.path.join(
		PROJECT_DIR,
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
	imgLists = utils.imgLists(imgDir, labelNames=(BOX_LABEL, NO_BOX_LABEL))
	print "Predictions for box images:\n{}".format(n2.predictFiles(imgLists[BOX_LABEL], imgDir))
	print "Predictions for non-box images:\n{}".format(n2.predictFiles(imgLists[NO_BOX_LABEL], imgDir))