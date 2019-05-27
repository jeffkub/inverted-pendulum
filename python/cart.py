import math

import numpy as np

class Cart:
	def __init__(self, params, sensors, motor):
		self.sensors = sensors
		self.motor = motor

		pulley_teeth = params['pulley_teeth']['val']
		belt_pitch = params['belt_pitch']['val']

		self.guard = params['guard']['val']
		self.pulley_radius = params['pulley_radius']['val']
		self.motor_torque_coeff = params['motor_torque_coeff']['val']
		self.motor_speed_coeff = params['motor_speed_coeff']['val']
		self.motor_start_torque = params['motor_start_torque']['val']

		self.x_scale = belt_pitch * pulley_teeth
		self.theta_scale = 2 * math.pi
		self.motor_angle_scale = 2 * math.pi

		self.limit = 0.0

		self.last_theta = 0.0
		self.last_x = 0.0
		self.last_motor_angle = 0.0

		self.motor_vel = 0.0

	# Unit is [m]
	def _getX(self):
		return self.sensors.getEncoder(0) * self.x_scale

	# Unit is [m]
	def _setXOffset(self, offset):
		self.sensors.setEncoderOffset(0, offset / self.x_scale)

	# Unit is [rad]
	def _getTheta(self):
		return self.sensors.getEncoder(1) * self.theta_scale

	# Unit is [rad]
	def _setThetaOffset(self, offset):
		self.sensors.setEncoderOffset(1, offset / self.theta_scale)

	# Unit is [rad]
	def _getMotorAngle(self):
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

	# Unit is [m]
	def goTo(self, position):
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

	# Unit is [s]
	def nextState(self, dt):
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

	# Unit is [N]
	def setForce(self, force):
		# Disable motor if zero
		if force == 0:
			self.motor.setMotorV(0)
			return

		torque = abs(force) * self.pulley_radius + self.motor_start_torque
		voltage = torque * self.motor_torque_coeff + self.motor_vel * self.motor_speed_coeff

		if force > 0:
			self.motor.setMotorV(voltage)
		else:
			self.motor.setMotorV(-voltage)
