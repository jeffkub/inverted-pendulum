import numpy as np

class Cart:
	def __init__(self, params, sensors, motor):
		self.sensors = sensors
		self.motor = motor

		self.guard = params['guard']['val']

		self.limit = 0.0

		self.last_theta = 0.0
		self.last_x = 0.0

	def findLimits(self):
		self.sensors.read()

		self.motor.setMotor(-0.4)
		while not self.sensors.getSwitch(0):
			self.sensors.read()

		x_min = self.sensors.getX()

		self.motor.setMotor(0.4)
		while not self.sensors.getSwitch(1):
			self.sensors.read()

		x_max = self.sensors.getX()

		self.motor.setMotor(0.0)

		self.limit = (x_max - x_min) / 2

		offset = (x_min + x_max) / 2
		self.sensors.setXOffset(-offset)

	def zeroTheta(self):
		self.sensors.read()

		offset = self.sensors.getTheta()
		self.sensors.setThetaOffset(-offset)

	def goTo(self, position):
		self.sensors.read()

		if (self.sensors.getX() - position) < -0.01:
			self.motor.setMotor(0.4)
			while (self.sensors.getX() - position) < -0.01:
				self.sensors.read()

		elif (self.sensors.getX() - position) > 0.01:
			self.motor.setMotor(-0.4)
			while (self.sensors.getX() - position) > 0.01:
				self.sensors.read()

		self.motor.setMotor(0.0)

	def checkLimits(self):
		return abs(self.last_x) < (self.limit - self.guard)

	def nextState(self, dt):
		self.sensors.read()

		x = self.sensors.getX()
		theta = self.sensors.getTheta()

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
		self.motor.setMotor(force)
