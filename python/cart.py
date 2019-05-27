import math

import numpy as np

class Cart:
	def __init__(self, params, sensors, motor):
		self.sensors = sensors
		self.motor = motor

		pulley_teeth = params['pulley_teeth']['val']
		belt_pitch = params['belt_pitch']['val']

		self.guard = params['guard']['val']			# [m]

		self.x_scale = belt_pitch * pulley_teeth	# [m/rev]
		self.theta_scale = 2 * math.pi				# [rad/rev]
		self.motor_angle_scale = 2 * math.pi 		# [rad/rev]

		self.limit = 0.0							# [m]

		self.last_theta = 0.0						# [rad]
		self.last_x = 0.0							# [m]
		self.last_motor_angle = 0.0					# [rad]

		self.motor_vel = 0.0						# [rad/s]

	def _getX(self):
		# Unit is [m]
		return self.sensors.getEncoder(0) * self.x_scale

	def _setXOffset(self, offset):
		# Unit is [m]
		self.sensors.setEncoderOffset(0, offset / self.x_scale)

	def _getTheta(self):
		# Unit is [rad]
		return self.sensors.getEncoder(1) * self.theta_scale

	def _setThetaOffset(self, offset):
		# Unit is [rad]
		self.sensors.setEncoderOffset(1, offset / self.theta_scale)

	def _getMotorAngle(self):
		# Unit is [rad]
		return self.sensors.getEncoder(0) * self.motor_angle_scale

	def findLimits(self):
		self.sensors.read()

		self.motor.setMotorV(-5)
		while not self.sensors.getSwitch(0):
			self.sensors.read()

		x_min = self._getX()

		self.motor.setMotorV(5)
		while not self.sensors.getSwitch(1):
			self.sensors.read()

		x_max = self._getX()

		self.motor.setMotorV(0)

		# Center the X axis origin
		offset = (x_min + x_max) / 2
		self._setXOffset(-offset)

		# Limit is +/- about the origin
		self.limit = (x_max - x_min) / 2

	def zeroTheta(self):
		self.sensors.read()

		offset = self._getTheta()
		self._setThetaOffset(-offset)

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
		return abs(self.last_x) <= (self.limit - self.guard)

	def resetState(self):
		self.sensors.read()

		self.motor_vel = 0.0

		self.last_x = self._getX()
		self.last_theta = self._getTheta()
		self.last_motor_angle = self._getMotorAngle()

	def nextState(self, dt):
		# Unit is [s]
		self.sensors.read()

		x = self._getX()
		theta = self._getTheta()
		motor_angle = self._getMotorAngle()

		# State vector is [theta, theta_dot, x, x_dot]
		state = np.array([
			theta,
			(theta - self.last_theta) / dt,
			x,
			(x - self.last_x) / dt
		])

		# Motor angular velocity
		self.motor_vel = (motor_angle - self.last_motor_angle) / dt

		self.last_x = x
		self.last_theta = theta
		self.last_motor_angle = motor_angle

		return state, x

	def setForce(self, force):
		# Unit is [N]
		# TODO: Implement motor equation
		self.motor.setMotorV(force)
