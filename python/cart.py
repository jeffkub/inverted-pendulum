import math

import numpy as np

class Cart:
	def __init__(self, params, sensors, motor):
		self.sensors = sensors
		self.motor = motor

		pulley_teeth = params['pulley_teeth']['val']
		belt_pitch = params['belt_pitch']['val']

		self.guard = params['guard']['val']

		self.x_scale = belt_pitch * pulley_teeth	# [m/rev]
		self.theta_scale = 2 * math.pi				# [rad/rev]

		self.limit = 0.0

		self.last_theta = 0.0
		self.last_x = 0.0

	def _getX(self):
		# Unit is [m]
		return self.sensors.getEncoder(0) * self.x_scale

	def _getTheta(self):
		# Unit is [rad]
		return self.sensors.getEncoder(1) * self.theta_scale

	def findLimits(self):
		self.sensors.read()

		self.motor.setMotorV(-5)
		while not self.sensors.getSwitch(0):
			self.sensors.read()

		x_min = self.sensors.getEncoder(0)

		self.motor.setMotorV(5)
		while not self.sensors.getSwitch(1):
			self.sensors.read()

		x_max = self.sensors.getEncoder(0)

		self.motor.setMotorV(0)

		offset = (x_min + x_max) / 2
		self.sensors.setEncoderOffset(0, -offset)

		self.limit = (x_max - x_min) / 2 * self.x_scale

	def zeroTheta(self):
		self.sensors.read()

		offset = self.sensors.getEncoder(1)
		self.sensors.setEncoderOffset(1, -offset)

	def goTo(self, position):
		# Unit is [m]
		self.sensors.read()

		if (self._getX() - position) < -0.01:
			self.motor.setMotorV(5)
			while (self._getX() - position) < -0.01:
				self.sensors.read()

		elif (self._getX() - position) > 0.01:
			self.motor.setMotorV(-5)
			while (self._getX() - position) > 0.01:
				self.sensors.read()

		self.motor.setMotorV(0)

	def checkLimits(self):
		return abs(self.last_x) < (self.limit - self.guard)

	def nextState(self, dt):
		# Unit is [s]
		self.sensors.read()

		x = self._getX()
		theta = self._getTheta()

		# State vector is [theta, theta_dot, x, x_dot]
		state = np.array([
			theta,
			(theta - self.last_theta) * dt,
			x,
			(x - self.last_x) * dt
		])

		self.last_x = x
		self.last_theta = theta

		return state, x

	def setForce(self, force):
		# Unit is [N]
		# TODO: Implement motor equation
		self.motor.setMotorV(force)
