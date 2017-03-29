import numpy as np
import serial
from time import sleep

class HXServo :
	#register addresses
	POSITION_REGADDR               	= 0x01
	SPEED_REGADDR           	= 0x02
	ADDR_REGADDR                	= 0x03
	BAUDRATE_REGADDR       		= 0x04
	COMMUNICATION_MODE_REGADDR 	= 0X21
	P_GAIN_REGADDR                	= 0x05
	D_GAIN_REGADDR                	= 0x06
	I_GAIN_REGADDR                	= 0x07
	PDI_GAIN_REGADDR              	= 0x08
	PDI_DEADBAND_REGADDR          	= 0x1f
	ACCELERATION_REGADDR          	= 0x20
	CW_FLEXIBLE_MARGIN_REGADDR    	= 0x09
	CCW_FLEXIBLE_MARGIN_REGADDR   	= 0x0a
	CW_FLEXIBLE_SLOPE_REGADDR     	= 0x0b
	CCW_FLEXIBLE_SLOPE_REGADDR    	= 0x0c
	WORKING_MODE_REGADDR          	= 0x0d
	SERVO_STATE_REGADDR           	= 0x0e
	MAX_POSITION_REGADDR          	= 0x0f
	MIN_POSITION_REGADDR          	= 0x10
	MAX_TEMPERATURE_REGADDR       	= 0x11
	MAX_VOLTAGE_REGADDR           	= 0x12
	MIN_VOLTAGE_REGADDR           	= 0x13
	MAX_TORQUE_REGADDR            	= 0x14
	ALL_WRITEREAD_REGADDR         	= 0x15
	CURRENT_POSITION_REGADDR      	= 0x16
	CURRENT_SPEED_REGADDR         	= 0x17
	CURRENT_TORQUE_REGADDR        	= 0x18
	CURRENT_VOLTAGE_REGADDR       	= 0x19
	CURRENT_TEMPERATURE_REGADDR   	= 0x1a
	ALL_READONLY_REGADDR          	= 0x1b
	FIRMWARE_VERSION_REGADDR      	= 0x1c
	MODEL_CODE_REGADDR            	= 0x1d
	WRITE_FLASH_REGADDR           	= 0x1e
	#setting status
	SETTING_OK       		= 0x01
	SETTING_FAIL     		= 0x02
	#cont				
	READ_REG         		= 0x01
	WRITE_REG       		= 0x02
	ANSWER           		= 0x03
	MASTER_BUSADDR   		= 0x0001
	BROADCAST_BUSADDR		= 1000
	def __init__(self, serialPort):
		self.serialPort = serialPort

	def write(self, devAddr, regAddr, data):
		'''write data to servo device'''
		checkSum = 0
		ret = 0
		if ((devAddr > self.BROADCAST_BUSADDR) or devAddr <= self.MASTER_BUSADDR):
			return ret
		buff = [0xaa, 0x55]	#frame header
		#master bus address, 2 bytes
		buff.append((self.MASTER_BUSADDR >> 8) & 0xff)
		buff.append(self.MASTER_BUSADDR & 0xff)
		#device address, 2 bytes
		buff.append((devAddr >> 8) & 0xff)
		buff.append(devAddr & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)
		#frame length, 1byte
		buff.append(10)
		#frame type: write_reg, 1 byte
		buff.append(self.WRITE_REG &0xff)
		if buff[-1] == 0xaa:
			buff.append(0)
		#register address, 1byte
		buff.append(regAddr & 0xff)
		if (buff[-1] == 0xaa):
			buff.append(0)
		#data, 2 bytes
		buff.append((data>>8) & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)
		buff.append(data & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)
		#checksum, 1 byte
		checksum = self.checkSumCal(buff, 2, len(buff))
		buff.append(checksum)
		if buff[-1] == 0xaa:
			buff.append(0)
		#tail
		buff.append(0xaa)
		buff.append(0x81)
			
		#send data by serial port
		sendData = ''
		for i in range(0, len(buff)):
			sendData += chr(buff[i])
		self.serialPort.write(sendData)
		#receive data
		receivedData = []
		tailReceived = False
		headerReceived = False
		while not(tailReceived and headerReceived):
			temp = self.serialPort.read(13)	#read a long enough data stream
			self.serialPort.flushInput()
			for i in range(0, len(temp)-1):
				if headerReceived:
					receivedData.append(ord(temp[i]))
				if ord(temp[i]) == 0xAA and ord(temp[i+1]) == 0x81:	#tail
					tailReceived = True
					receivedData.append(ord(temp[i+1]))
				if ord(temp[i]) == 0xAA and ord(temp[i+1]) == 0x55:	#header
					headerReceived = True
					receivedData.append(ord(temp[i]))
		if len(receivedData) == 0:
			return 0
		index = len(receivedData)
		
		if ((index >= 13) and (receivedData[0] == 0xaa) and (receivedData[1] == 0x55) 
			and (receivedData[index - 1] == 0x81) and (receivedData[index - 2] == 0xaa)):
			#delete frame header and tail
			buffData = []
			for i in xrange(2, index - 2):
				buffData.append(receivedData[i])
				if receivedData[i] == 0xaa:
					continue

			if((((buffData[0]<<8)|(buffData[1]))==devAddr) and (((buffData[2]<<8)|(buffData[3]))== self.MASTER_BUSADDR)):
				if((buffData[4]==9) and (buffData[5]==self.ANSWER) and (buffData[6]==regAddr)):
					checksum = self.checkSumCal (buffData, 0, 8);
				if((checksum==buffData[8]) and (buffData[7]== self.SETTING_OK)):
					ret = self.SETTING_OK;
		return ret

	def read(self, devAddr, regAddr):
		'''read data from servo device'''
		checkSum = 0
		ret = 0

		if ((devAddr > self.BROADCAST_BUSADDR) or devAddr <= self.MASTER_BUSADDR):
			return ret
		buff = [0xaa, 0x55]	#frame header
		#master bus address, 2 bytes
		buff.append((self.MASTER_BUSADDR >> 8) & 0xff)
		buff.append(self.MASTER_BUSADDR & 0xff)
		#device address, 2 bytes
		buff.append((devAddr >> 8) & 0xff)
		buff.append(devAddr & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)
		#frame length, 1byte
		buff.append(8)
		#frame type: write_reg, 1 byte
		buff.append(self.READ_REG &0xff)
		if buff[-1] == 0xaa:
			buff.append(0)
		#register address, 1byte
		buff.append(regAddr & 0xff)
		if (buff[-1] == 0xaa):
			buff.append(0)
		#checksum, 1 byte
		checksum = self.checkSumCal(buff, 2, len(buff))
		buff.append(checksum)
		if buff[-1] == 0xaa:
			buff.append(0)
		#tail
		buff.append(0xaa)
		buff.append(0x81)
		
		#send data by serial port
		sendData = ''
		for i in range(0, len(buff)):
			sendData += chr(buff[i])
		self.serialPort.write(sendData)
		#receive data
		receivedData = []
		tailReceived = False
		headerReceived = False
		while not(tailReceived and headerReceived):
			temp = self.serialPort.read(14)	#read a long enough data stream
			#self.serialPort.flushInput()
			for i in range(0, len(temp)-1):
				if headerReceived:
					receivedData.append(ord(temp[i]))
				if ord(temp[i]) == 0xAA and ord(temp[i+1]) == 0x81:	#tail
					tailReceived = True
					receivedData.append(ord(temp[i+1]))
				if ord(temp[i]) == 0xAA and ord(temp[i+1]) == 0x55:	#header
					headerReceived = True
					receivedData.append(ord(temp[i]))

		if len(receivedData) == 0:
			return 0
		index = len(receivedData)
		if ((index >= 13) and (receivedData[0] == 0xaa) and (receivedData[1] == 0x55) 
			and (receivedData[index - 1] == 0x81) and (receivedData[index - 2] == 0xaa)):
			#delte frame header and tail
			buffData = []
			for i in xrange(2, index - 2):
				buffData.append(receivedData[i])
				if receivedData[i] == 0xaa:
					continue
			if((((buffData[0]<<8)|(buffData[1]))==devAddr) and (((buffData[2]<<8)|(buffData[3]))==self.MASTER_BUSADDR)):
				
				if((buffData[4]==10) and (buffData[5]==self.ANSWER) and (buffData[6]==regAddr)):
					checksum = self.checkSumCal (buffData, 0, 9);
				if(checksum==buffData[9]):
					ret = (buffData[7] << 8) | buffData[8];
		return ret

	def checkSumCal(self, data, start, end):
		check = 0x00 
		for x in xrange(start, end):
			check = int(check) ^ int(data[x])
		return check
	#get register value
	def getBaudRate(self, id):
		return self.read(id, self.BAUDRATE_REGADDR)
	def getWorkingMode(self, id):
		return self.read(id, self.WORKING_MODE_REGADDR)
	def getPosition(self, id):
		return self.read(id, self.POSITION_REGADDR)
	def getSpeed(self, id):
		return self.read(id, self.SPEED_REGADDR)
	def getCurrentPosition(self, id):
		return self.read(id, self.CURRENT_POSITION_REGADDR)
	def getCurrentTemperature(self, id):
		return self.read(id, self.CURRENT_TEMPERATURE_REGADDR)
	def getCurrentVoltage(self, id):
		return self.read(id, self.CURRENT_VOLTAGE_REGADDR)
	def getMinPosition(self, id):
		return self.read(id, self.MIN_POSITION_REGADDR)
	def getMaxPosition(self, id):
		return self.read(id, self.MAX_POSITION_REGADDR)
	def getMaxTorque(self, id):
		return self.read(id, self.MAX_TORQUE_REGADDR)
	def getMinVoltage(self, id):
		return self.read(id, self.MIN_VOLTAGE_REGADDR)
	def getMaxVoltage(self, id):
		return self.read(id, self.MAX_VOLTAGE_REGADDR)
	def getMaxTemperature(self, id):
		return self.read(id, self.MAX_TEMPERATURE_REGADDR)

	#setting functions
	def writeFlash(self, id):
		return self.write(id, self.WRITE_FLASH_REGADDR, 1)
	def setBaudRate(self, id, value):
		self.write(id, self.BAUDRATE_REGADDR, value)
		self.writeFlash(id)
	def setWorkingMode(self, id, value):
		self.write(id, self.WORKING_MODE_REGADDR, value)
	def setPosition(self, id, value):
		self.write(id, self.POSITION_REGADDR, value)
	def setSpeed(self, id, value):
		self.write(id, self.SPEED_REGADDR, value)
		self.writeFlash(id)
	def setMinPosition(self, id, value):
		self.write(id, self.MIN_POSITION_REGADDR, value)
		self.writeFlash(id)
	def setMaxPosition(self, id, value):
		self.write(id, self.MAX_POSITION_REGADDR, value)
		self.writeFlash(id)
	def setMinTorque(self, id, value):
		self.write(id, self.MIN_TORQUE_REGADDR, value)
		self.writeFlash(id)
	def setMaxTorque(self, id, value):
		self.write(id, self.MAX_TORQUE_REGADDR, value)
		self.writeFlash(id)
	def setMinVoltage(self, id, value):
		self.write(id, self.MIN_VOLTAGE_REGADDR, value)
		self.writeFlash(id)
	def setMaxVoltage(self, id, value):
		self.write(id, self.MAX_VOLTAGE_REGADDR, value)
		self.writeFlash(id)
	def setMaxTemperature(self, id, value):
		self.write(id, self.MAX_TEMPERATURE_REGADDR, value)
		self.writeFlash(id)
	def setAddr(self, id, value):
		self.write(id, self.ADDR_REGADDR, value)
		self.writeFlash(id)
#test code
if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyAMA0', 115200, timeout = 0.5)
	servo = HXServo(ser)
	id = 6
#	print "Read testing..."
#	servo.setWorkingMode(id, 1)
	print "BaudRate:", servo.getBaudRate(id)
#	print "Position:", servo.getPosition(id)
#	print "Speed:", servo.getSpeed(id)
#	print "CurrentPostion:", servo.getCurrentPosition(id)
#	print "CurrentTemperature:", servo.getCurrentTemperature(id)
#	print "CurrentVoltage:", servo.getCurrentVoltage(id)
#	print "MinPosition:", servo.getMinPosition(id)
#	print "MaxPosition:", servo.getMaxPosition(id)
#	print "MaxTorque:", servo.getMaxTorque(id)
#	print "MinVoltage:", servo.getMinVoltage(id)
#	print "MaxVoltage:", servo.getMaxVoltage(id)
#	print "MaxTemperature:", servo.getMaxTemperature(id)
#	servo.setWorkingMode(id, 4)
#	sleep(1)
#	servo.setWorkingMode(id, 5)
#	sleep(1)
#	servo.setWorkingMode(id, 1)
	for i in range(2, 10):
		servo.setWorkingMode(i,2)
		servo.setPosition(i, 2000)
		sleep(0.5)
		print "Position:", servo.getPosition(id)
	 	
#	servo.setAddr(id, 9)
#	servo.setMaxTemperature(id, servo.getMaxTemperature(id) + 10)
#	servo.setSpeed(id, servo.getSpeed(id) + 10)
#	servo.setMinPosition(id, servo.getMinPosition(id) - 20)
#	servo.setMaxPosition(id, servo.getMaxPosition(id) - 20)
#	servo.setMinVoltage(id, servo.getMinVoltage(id) + 10)
#	servo.setMaxVoltage(id, servo.getMaxVoltage(id) + 10)
#	servo.setMaxTorque(id, servo.getMaxTorque(id) + 10)
