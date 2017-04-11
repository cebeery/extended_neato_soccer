#!/usr/bin/env python

"""
Modified from Nick Francisci's Computational Robotics Warmup Repository:
https://github.com/ManickYoj/warmup_project_2017/blob/master/scripts/automaton.py
"""

import rospy, utils
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from sensor_msgs.msg import Image

class Automaton(object):
	def __init__(
	    self,
	    name="automaton",
	    states={},
	    initialState=None,
		debug=False,
	):
		rospy.init_node(name)

		# Setup sensor subscribers
		rospy.Subscriber('/bump', Bump, self.processBump)
		rospy.Subscriber('/camera/image_raw', Image, self.processImage)

		# Setup command publisher
		self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.states = states
		self.state = initialState
		self.debug = debug

	def processBump(self, bump):
		self.states[self.state].processBump(bump)

	def processImage(self, img):
		self.states[self.state].processImage(scan)

	def debugLog(self, msg):
		if self.debug:
			print msg

	def main(self):
		r = rospy.Rate(10)
		rospy.on_shutdown(lambda: self.cmd.publish(Twist()))
		while not rospy.is_shutdown():
			instructions = self.states[self.state].update()

			# Execute movement commands
			self.cmd.publish(instructions["command"])

			# Change state if necessary
			if instructions["changeState"] is not None:
				self.state = instructions["changeState"]
				self.states[self.state]._resume(instructions["changeArgs"])
				continue

			r.sleep()

class Behavior(object):
	def __init__(self, name="behavior", debug=False):
		self.debug = debug
		self.stateName = name
		self.linear = 0
		self.angular = 0
		self.nextState = None
		self.changeArgs = {}

	def _resume(self, args):
		self.nextState = None
		self.changeArgs = {}
		self.resume(args)

	def resume(self, args):
		pass

	def processBump(self, bump):
		pass

	def processImage(self, img):
		pass

	def setLinear(self, linear):
		self.linear = linear

	def setAngular(self, angular):
		self.angular = angular

	def setSpeed(self, linear, angular):
		self.linear = linear
		self.angular = angular

	def changeState(self, newState, args={}):
		self.nextState = newState
		self.changeArgs = args
		self.debugLog("Changing state to {}".format(newState))

	def update(self):
		return {
			"changeState": self.nextState,
			"changeArgs": self.changeArgs,
			"command": Twist(
				linear=Vector3(self.linear, 0, 0),
				angular=Vector3(0, 0, self.angular)
				)
		}	

	def debugLog(self, msg):
		if self.debug:
			print msg

if __name__ == "__main__":
pass
