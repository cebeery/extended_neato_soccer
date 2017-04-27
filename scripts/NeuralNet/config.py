#!/usr/bin/env python

"""
config.py
--------

Loads the config and makes it accessible via the CONFIG variable.

Example usage at bottom of file.

"""

import rospkg
import ConfigParser
import os

class Config(object):
	def __init__(self, filepath='config.ini'):
		r = rospkg.RosPack()
		filepath = os.path.join(
			r.get_path('extended_neato_soccer'),
			'scripts/NeuralNet',
			filepath
		)

		parser = ConfigParser.ConfigParser()
		parser.read(filepath)
		self.data = {}

		for section in parser.sections():
			for option in parser.options(section):
				datum = parser.get(section, option)

				if section == "INTS":
					self.data[option] = int(datum)
				else:
					self.data[option] = datum

	def get(self, key):
		key = key.lower()
		if key in self.data:
			return self.data[key]
		else:
			raise ValueError("Key '{}' not in config file.".format(key))

# Constants
CONFIG = Config()

if __name__ == "__main__":
	# Retrieve the config variable "ORIG_IMAGE_SIZE"
	print CONFIG.get("ORIG_IMAGE_SIZE")