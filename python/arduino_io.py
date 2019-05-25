import serial

class ArduinoIO:
	def __init__(self, params, port, baud=115200):
		self.x_scale = params['x_scale']['val']
		self.theta_scale = params['theta_scale']['val']
		self.motor_scale = params['motor_scale']['val']
		self.motor_offset = params['motor_offset']['val']

		self.x = 0.0
		self.x_offset = 0.0

		self.theta = 0.0
		self.theta_offset = 0.0

		self.switches = [0] * 2

		self.serial = serial.Serial(port, baud, timeout=0.2)

	def __del__(self):
		self.serial.close()

	def getX(self):
		return self.x + self.x_offset

	def setXOffset(self, offset):
		self.x_offset += offset

	def getTheta(self):
		return self.theta + self.theta_offset

	def setThetaOffset(self, offset):
		self.theta_offset += offset

	def getSwitch(self, index):
		return self.switches[index]

	def read(self):
		self.serial.reset_input_buffer()

		msg = 'R\n'
		self.serial.write(msg.encode('utf-8'))

		response = self.serial.readline().decode('utf-8')
		vals = response.split('\t')

		if len(vals) != 4:
			print('Invalid response')
			return

		self.x = float(vals[0]) * self.x_scale
		self.theta = float(vals[1]) * self.theta_scale

		self.switches[0] = int(vals[2]) > 0
		self.switches[1] = int(vals[3]) > 0

	def setMotor(self, force):
		if force > 0:
			force += self.motor_offset
		elif force < 0:
			force -= self.motor_offset

		val = int(force / self.motor_scale)
		clamped = max(-2047, min(2047, val))

		msg = 'M1: ' + str(clamped) + '\n'
		self.serial.write(msg.encode('utf-8'))
