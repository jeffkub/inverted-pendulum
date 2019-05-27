import serial

class ArduinoIO:
	def __init__(self, params, port, baud=115200):
		self.encoder_scale = params['encoder_scale']['val']
		self.motor_v_scale = params['motor_v_scale']['val']

		self.encoder = [0.0] * 2
		self.encoder_offset = [0.0] * 2

		self.switch = [False] * 2

		self.serial = serial.Serial(port, baud, timeout=0.2)

	def __del__(self):
		self.serial.close()

	# Unit is [revs]
	def getEncoder(self, index):
		return self.encoder[index] + self.encoder_offset[index]

	# Unit is [revs]
	def setEncoderOffset(self, index, offset):
		self.encoder_offset[index] += offset

	# True if switch is activated, else False
	def getSwitch(self, index):
		return self.switch[index]

	def read(self):
		self.serial.reset_input_buffer()

		msg = 'R\n'
		self.serial.write(msg.encode('utf-8'))

		response = self.serial.readline().decode('utf-8')
		vals = response.split('\t')

		if len(vals) != 4:
			print('Invalid response')
			return

		self.encoder[0] = float(vals[0]) / self.encoder_scale
		self.encoder[1] = float(vals[1]) / self.encoder_scale

		self.switch[0] = int(vals[2]) > 0
		self.switch[1] = int(vals[3]) > 0

	# Unit is [V]
	def setMotorV(self, voltage):
		val = int(voltage / self.motor_v_scale)
		clamped = max(-2047, min(2047, val))

		msg = 'M1: ' + str(clamped) + '\n'
		self.serial.write(msg.encode('utf-8'))
