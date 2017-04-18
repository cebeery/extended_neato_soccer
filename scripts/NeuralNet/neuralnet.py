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
import numpy as np
import Image

class Network(NeuralNet):
	def __init__(self):

		# Configure neural net
		super(Network, self).__init__(
			# Topography of network
			layers=[
				('input', layers.InputLayer),
				('hidden', layers.DenseLayer),
				('output', layers.DenseLayer),
			],

			# Layer parameters
			input_shape=(None, 128, 128),		# Shape of the input (# imgs, rows, cols)
			hidden_num_units=100,						# Number of units in each hidden layer
			output_nonlinearity=None,				# BLACKBOX: Output uses identity function
			output_num_units=1,							# Target values (Ball: y/n, x-coord, y-coord, radius)

			# Optimization method
			update=nesterov_momentum,				# BLACKBOX
			update_learning_rate=0.01,			# BLACKBOX
			update_momentum=0.9,						# BLACKBOX

			regression=True,								# BLACKBOX
			max_epochs=200,									# BLACKBOX
			verbose=0,											# Print output trace or not during training
		)

	def trainWith(self, imgsPath, presentPrefix):
		# Get a list of all of the image paths from the imgPath directory
		imgNames = os.listdir(imgsPath)
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
	n = Network()
	n.trainWith('./training-imgs/example', presentPrefix="boxes")

	print "Avg prediction for box images: {}".format(
		n.predict([
			'./training-imgs/example/boxes_00{}.png'.format(i)
			for i in range(6)
		])
	)

	print "Avg prediction for non-box images: {}".format(
		n.predict([
			'./training-imgs/example/no_boxes_00{}.png'.format(i)
			for i in range(10)
		])
	)