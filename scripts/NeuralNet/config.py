#!/usr/bin/env python

"""
config.py
--------

Loads the config and makes it accessible via the CONFIG object.

Example usage at bottom of file.

"""

import rospkg
import ConfigParser
import os


class Config(object):
	"""
	A wrapper to contain all cross cutting constants, most of which are loaded
	from a config file and some of which must be determined at runtime.

	Arguably, should be a singleton, but is not.

	"""

	def __init__(self, filename='config.ini'):
		"""
		Set up the config with all the data needed.

		Inputs:
			filename (string): the config file to load from.

		"""

		# Determine filepaths
		r = rospkg.RosPack()
		projectDir = r.get_path('extended_neato_soccer')
		filepath = os.path.join(
			projectDir,
			'scripts/NeuralNet',
			filename
		)

		parser = ConfigParser.ConfigParser()
		parser.read(filepath)

		# Create runtime config entries
		self.data = {
			'project_dir': projectDir,
		}

		# Load config file information into config object memory
		for section in parser.sections():
			for option in parser.options(section):
				datum = parser.get(section, option)

				if section == "INTS":
					self.data[option] = int(datum)
				if section == "FLOATS":
					self.data[option] = float(datum)
				elif section == "STRINGS":
					self.data[option] = datum


	def get(self, key):
		"""
		Retrieve a constant from this config object.

		Input:
			key (string): the name of the constant to retrieve.

		Output:
			(*) the value of the requested constant

		"""

		key = key.lower()
		if key in self.data:
			return self.data[key]
		else:
			raise ValueError("Key '{}' not in config file.".format(key))

# Create a constant CONFIG object which can be loaded and referenced from other
# modules. This is expected use.
CONFIG = Config()

# Example usage
if __name__ == "__main__":
	# Retrieve the config variable "ORIG_IMAGE_SIZE"
	print CONFIG.get("ORIG_IMAGE_SIZE")