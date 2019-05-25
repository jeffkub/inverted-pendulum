import json
import sys
import time
import traceback

import numpy as np

from arduino_io import ArduinoIO
from cart import Cart

class Controller:
	def __init__(self, params, sensors, motor):
		self.dt = params['dt']['val']
		self.Khat = np.array(params['Khat']['val'])

		self.cart = Cart(params, sensors, motor)

		self.r = 0.0
		self.zeta = 0.0

	def _waitForNextStep(self):
		time.sleep(max(0, self.dt - (time.time() - self.last_time)))
		self.last_time += self.dt

	def _step(self):
		# Get the current state of the cart
		[state, y] = self.cart.nextState(self.dt)

		# Zeta is the integral of the position error
		self.zeta += (self.r - y) * self.dt

		# The control equation
		u = np.dot(-self.Khat, np.append(state, self.zeta))

		self.cart.setForce(u)

	def setup(self):
		# Initialize encoder positions
		self.cart.zeroTheta()
		self.cart.findLimits()

	def run(self):
		# Center the cart
		self.cart.goTo(0.0)

		# Wait for the pendulum to be moved into position by hand
		# TODO

		# Control loop
		self.last_time = time.time()
		self.zeta = 0.0

		self.cart.nextState(self.dt)

		while True:
			self._waitForNextStep()
			self._step()

			# Make sure the cart doesn't crash into the ends
			if not self.cart.checkLimits():
				break

def main(argv):
	with open('params.json') as file:
		params = json.load(file)

	arduino = ArduinoIO(params, '/dev/ttyACM0')
	controller = Controller(params, arduino, arduino)

	try:
		controller.setup()

		while True:
			controller.run()
	except Exception:
		print(traceback.format_exc())

	# Make sure to disable the motor before exiting
	arduino.setMotor(0)

	return 0

if __name__ == '__main__':
	sys.exit(main(sys.argv))
