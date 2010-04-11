# -*- coding: utf-8 -*-
# Pybp interface to BusPirate
# Copyright (C) 2010  Paeae Technologies
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import serial
import time


#exceptions
class BusPirateError(Exception):
	pass
	
class CommandError(Exception):
	pass
	
class SerialError(Exception):
	pass

#protocol mode definitions
class Modes:
	UART = "uart"
	SPI = "spi"
	I2C = "i2c"
	ONEWIRE = "1wire"
	RAW = "raw"

class Commands:
	RESET_BINARY = chr(0b0)
	RESET_USER = chr(0b1111)

	#protocol modes
	ENTER_SPI = chr(0b1)
	ENTER_I2C = chr(0b10)
	ENTER_UART = chr(0b11)
	ENTER_1WIRE = chr(0b100)
	ENTER_RAW = chr(0b101)

	#uart mode commands
	UART_START_ECHO = chr(0b10)
	UART_STOP_ECHO = chr(0b11)
	UART_WRITE = chr(0b10000)
	UART_SET_BAUDRATE = chr(0b01100000)
	UART_SET_CONFIG = chr(0b10000000)	
	UART_SET_PINS = chr(0b01000000)
	UART_ENTER_BRIDGE = chr(0b1111)
	
	#tests
	TEST_SHORT = chr(0b10000)
	TEST_LONG = chr(0b10001)
	TEST_EXIT = chr(0xff)
	#pwm
	PWM_SET = chr(0b10010)
	PWM_CLEAR = chr(0b10011)

	#probes
	PROBE_VOLTAGE = chr(0b10100)

class Responses:

	RESET_BINARY = 'BBIO1'
	ENTER_SPI = 'SPI1'
	ENTER_I2C = 'I2C1'
	ENTER_UART = 'ART1'
	ENTER_1WIRE = '1W01'
	ENTER_RAW = 'RAW1'

	OK = chr(0x1) #generic OK response for most commands

class CommandResponsePairs:
	RESET_BINARY = (Commands.RESET_BINARY,Responses.RESET_BINARY)
	ENTER_SPI = (Commands.ENTER_SPI,Responses.ENTER_SPI)
	ENTER_UART = (Commands.ENTER_UART,Responses.ENTER_UART)
	ENTER_I2C = (Commands.ENTER_I2C,Responses.ENTER_I2C)
	ENTER_1WIRE = (Commands.ENTER_1WIRE,Responses.ENTER_1WIRE)
	ENTER_RAW = (Commands.ENTER_RAW,Responses.ENTER_RAW)
	
	TEST_EXIT = (Commands.TEST_EXIT,Responses.OK)

	PWM_CLEAR = (Commands.PWM_CLEAR,Responses.OK)
	
	
class BusPirate(object):
	"""BusPirate interface class"""
	
	def __init__(self, port,**kwargs):
		"""arguments: 
			port -- serial port to use
			
			keyword arguments:
				baudrate -- baudrate to use, defaults to 115200
		"""
		super(BusPirate, self).__init__()

		#use dummy serial device in unittests
		self.unittestSerial = kwargs.pop('unittestserial',None)

		self.port = port
		self.baudrate = kwargs.pop('baudrate',115200)
		self.serial = None
		self.open()
		
	"""Opens connection to BusPirate and enters binary mode. Closes existing connection if already open"""
	def open(self):
		self.close()
		self.serial = self.unittestSerial or serial.Serial(self.port,self.baudrate)
		self.serial.timeout = 2.0 #seconds
		self.lastresponse=None
		self.mode = None
		self._uartEcho = False
		
		self._enterBinaryMode()
		
	"""Closes connection to BusPirate"""
	def close(self):
		if self.serial:
			self._sendCmd(Commands.RESET_USER)
			self.serial.close()
			self.serial=None
	
	def selfTest(self,longTest=False):
		"""Perform self-test
			arguments:
				longTest -- Perform long self-test. Requires jumpers between +5 and Vpu, +3.3 and ADC. Defaults to False
		"""
		if longTest:
			self._sendCmd(Commands.TEST_LONG)
		else:
			self._sendCmd(Commands.TEST_SHORT)
			
		result = None
		failed = True
		for i in range(10):
			result = self._read(1)
			if len(result) > 0:
				failed = False
				break
				
		if failed:
			raise BusPirateError("Selftest timeout")
		
		
		self._sendCmd(*CommandResponsePairs.TEST_EXIT)
		return result[0] == chr(0)
		
	def enterMode(self,mode=""):
		"""Enter protocol mode
	
			Arguments:
				mode -- "spi","uart","i2c","1wire","raw"
		"""
		mode = mode.lower()
		
		try:
			pair = None #command and response pair
			
			if mode == Modes.UART:
				pair = CommandResponsePairs.ENTER_UART
			elif mode == Modes.SPI:
				pair = CommandResponsePairs.ENTER_SPI
			elif mode == Modes.I2C:
				pair = CommandResponsePairs.ENTER_I2C
			elif mode == Modes.ONEWIRE:
				pair = CommandResponsePairs.ENTER_1WIRE
			elif mode == Modes.RAW:
				pair = CommandResponsePairs.ENTER_RAW
			else:
				raise BusPirateError("Invalid mode '%s'" % mode)

			cmd,response = pair
			self._sendCmd(cmd,response)
			self.mode = mode
			
		except CommandError as err:
			raise BusPirateError(err)

		return True
		
	def leaveMode(self):
		"""Leave protocol mode"""
		if not self.mode:
			raise BusPirateError("Not in protocol mode")
			
		self._sendCmd(*CommandResponsePairs.RESET_BINARY)
		self.mode = None
		return True
		
	#pwm
	def setPWM(self,dutycycle,hz):
		"""Set AUX pin PWM
			Arguments:
				duty -- dutycycle 0.0 - 1.0
				hz -- pwm frequency
		"""
		prescaler = 0
		period = 0
		div = 0
		#adapted from bus pirate sources
		if hz < 4:
			prescaler=0b11
			div = 62
		elif hz < 31:
			div = 250
			prescaler = 0b10
		elif hz < 245:
			div = 2000
			prescaler = 0b01
		else:
			div = 16000
		
		if dutycycle < 0.0:
			dutycycle = 0.0
		elif dutycycle > 1.0:
			dutycycle=1.0
		
		period = int((div / hz)-1)
		duty = int(period  * dutycycle)
		
		cmd = [ Commands.PWM_SET, chr(prescaler),chr(duty>>8),chr(duty&0xff),chr(period>>8),chr(period&0xff)]
		self._sendCmd("".join(cmd),Responses.OK)
		return True
		
	def stopPWM(self):
		"""Stop outputting PWM"""
		self._sendCmd(*CommandResponsePairs.PWM_CLEAR)
		return True
		
	#uart mode commands
	def uartStartEcho(self):
		"""Start uart echo mode. Received data can be """
		self._checkMode(Modes.UART)
		self._sendCmd(*CommandResponsePairs.UART_START_ECHO)
		self._uartEcho = True
		return True
		
	def uartStopEcho(self):
		self._checkMode(Modes.UART)
		self._sendCmd(*CommandResponsePairs.UART_STOP_ECHO)
		return True
		
	def uartWrite(self,data):
		"""Writes data to uart"""
		self._checkMode(Modes.UART)
		if not self._uartEcho:
			raise BusPirateError("Not in UART echo mode")
		
		#Up to 16 data bytes can be sent at once.
		#TODO: implement >16byte transfer
		if len(data) <= 16:
			cmd = chr(ord(Commands.UART_WRITE) | (len(data)-1)) #Note that 0000 indicates 1 byte because there’s no reason to send 0
			self._sendCmd(cmd,Responses.OK)
			
			for c in data:
				self._write(c)
				if self._read(1) != response:
					raise BusPirateError("uartWrite received invalid response")
		else:
			raise NotImplementedError(">16byte uartwrite")

		return True
	
	def uartBytesAvailable(self):
		self._checkMode(Modes.UART)
		if not self._uartEcho:
			raise BusPirateError("Not in UART echo mode")

		return self._available()
		
	def uartReceive(self,size=1):
		self._checkMode(Modes.UART)
		if not self._uartEcho:
			raise BusPirateError("Not in UART echo mode")

		return self._read(size)
		
	def uartSetSpeed(self,baudrate=9600):
		"""configures uart baudrate"""
		self._checkMode(Modes.UART)

		rates = { 300 : 0,  1200 : 0b0001, 2400: 0b0010,4800 : 0b0011,9600 : 0b0100,19200: 0b0101,31250 : 0b0110, 38400 : 0b0111,57600 : 0b1000,115200 : 0b1010}
		
		if baudrate not in rates:
			raise BusPirateError("Invalid baudrate '%d'" % baudrate)
		
		self._sendCmd(chr(ord(Commands.UART_SET_BAUDRATE)|rates[baudrate]),Responses.OK)
		return True
		
	def uartSetPins(self,**kwargs):
		"""Configure peripherals.
			Keyword arguments:
				power  -- defaults to 0
				pullups -- defaults to 0
				aux -- defaults to 0
				cs -- defaults to 0
		"""
		self._checkMode(Modes.UART)
		
		power = kwargs.pop("power",0)
		pullups = kwargs.pop("pullups",0)
		aux = kwargs.pop("aux",0)
		cs = kwargs.pop("cs",0)
		cmd = chr(ord(Commands.UART_SET_PINS) | (power << 3) | (pullups << 2) | (aux << 1) | cs)
		
		self._sendCmd(cmd,Responses.OK)
		return True
		
	def uartSetConfig(self,**kwargs):
		"""Set UART configuration.
			Keyword arguments:
				output -- 0 = HiZ, 1 = 3.3v, defaults to HiZ (0)
				databits -- 8 or 9, defaults to 8
				parity -- 'N' (none) or 'E' (even) or 'O' (odd), defaults to 'N'
				stopbits -- stop bits , defaults to 1
				polarity -- 0 = idle high, 1 = idle low, defaults to idle high (0)
		"""
		self._checkMode(Modes.UART)
		
		output = kwargs.pop("output",0)
		databits = kwargs.pop("databits",8)
		parity = kwargs.pop('parity','N').lower()
		stopbits = kwargs.pop("stopbits",1)
		polarity = kwargs.pop("polarity",0)
		
		#verify parameters
		if output != 0 and output != 1:
			raise BusPirateError("Output value is invalid")
		if stopbits != 0 and stopbits != 1:
			raise BusPirateError("Stopbits value is invalid")
		if polarity != 0 and polarity != 1:
			raise BusPirateError("Polarity is invalid")
		
		#combine databits and parity
		dp = 0
		if databits == 9:
			if parity == 'e':
				raise BusPirateError("Parity cannot be even if databits is 9")
			else:
				dp = 3
		elif databits == 8:
			if parity == 'n':
				dp = 0
			elif parity == 'e':
				dp = 1
			elif parity == 'o':
				dp = 2
		else:
			raise BusPirateError("Databits value is invalid.")
		
		self._sendCmd(chr(ord(Commands.UART_SET_CONFIG) | (output<<4) | (dp << 3) | (stopbits << 1) | polarity),Responses.OK)
		return True
		
	def uartBridgeMode(self):
		"""Enters UART bridge mode. NOTE: Bridge mode cannot be exited programmatically, Bus Pirate has to be reseted manually.
			Returns: serial instance which can be used directly
		"""
		self._checkMode(Modes.UART)
		self._sendCmd(Commands.UART_ENTER_BRIDGE)
		return self.serial
		
		
	#internal functions
	def _enterBinaryMode(self,short=False):
		#From the manual: Send 0x00 to the user terminal 20 times to enter the raw binary bitbang mode.
		#				One way to ensure that you're at the command line is to send <enter> at least 10 times, 
		#				and then send '#' to reset. Next, send 0x00 to the command line 20+ times 
		#				until you get the BBIOx version string.
		if short:
			self._sendCmd(*CommandResponsePairs.RESET_BINARY)
		else:
			try:
				#TODO: this is a very naive version,make it better
				for i in range(10):
					self._write('\n')
					time.sleep(0.001)
					
				self._write('#\n')
				time.sleep(0.001)
				
				for i in range(25):
					self._write(chr(0x0))
					time.sleep(0.001)
				#read binary mode protocol version
				time.sleep(0.5)
				result = self._read(self._available())

				if result and result.endswith(Responses.RESET_BINARY):
					return True

				raise BusPirateError("Failed to enter binary mode")
			except SerialError:
				raise BusPirateError("Serial exception raised while trying to enter binary mode")

	def _getResponse(self):
		return self.lastresponse

	def _sendCmd(self,cmd,expect=None):
		self._write(cmd)
		if expect:
			time.sleep(0.001) #maybe not necessary
			response = self._read(len(expect))
			self.lastresponse = response
			if response != expect:
				raise CommandError("Sent command %s and expected return '%s' but received '%s'" % (hex(ord(cmd)),expect,response))

		return True

	def _available(self):
		return self.serial.inWaiting()
		
	def _read(self,length):
		try:
			ret = self.serial.read(length)
		except: 
			return None
		return ret

	def _write(self,data):
		try:
			ret = self.serial.write(data)
		except serial.SerialTimeoutException:
			raise SerialError("_write timeout")
		return ret
		
	def _checkMode(self,mode):
		if not self.mode or self.mode != mode:
			raise BusPirateError("Not in protocol mode '%s'" % mode)
	
	def _printhex(self,data):
		s = [hex(ord(x)) for x in data]
		print s
		
if __name__ == '__main__':
	import unittest
	import re
	device = None
	#simulates bus pirate
	class MockBP(serial.Serial):
		def __init__(self,**kwargs):
			super(MockBP,self).__init__(None)
			self.input = ""
			self.output = ""
			self.args = kwargs
			self.binmode = False
			
		def write(self,data):
			self.input += data
			self.processInput()
			
		def read(self,size=1):
			if len(self.output) < size:
				size = len(self.output)
			
			data = self.output[:size]
			self.output = self.output[size:]
			return data
		
		def _output(self,data):
			self.output += data
		
		def inWaiting(self):
			return len(self.output)
		
		def processInput(self):
			if not self.binmode:
				if "failbinmode" in self.args:
					return
				if re.search('\000{20}$',self.input):
					self._output('BBIO1')
					self.binmode=True
			else:
				cmd = self.input[-1:]
				if "failmode" not in self.args:
					if cmd == Commands.ENTER_UART:
						self._output('ART1')
					elif cmd == Commands.ENTER_SPI:
						self._output('SPI1')
	
	#tests that use mock object
	class BPTestMock(unittest.TestCase):
		def testFailBinmode(self):
			self.assertRaises(BusPirateError,lambda: BusPirate(None,unittestserial=MockBP(failbinmode=True)))
			
		def testSuccessBinmode(self):
			BusPirate(None,unittestserial=MockBP())
			
		def testInvalidModes(self):
			modes = ["adjhf","fjskd","aurt","isp"]
			bp = BusPirate(None,unittestserial=MockBP())
			
			for mode in modes:
				self.assertRaises(BusPirateError,bp.enterMode,mode)
				
		def testUartMode(self):
			bp = BusPirate(None,unittestserial=MockBP())
			bp.enterMode('UART')

	#tests that use real device
	class BPTestReal(unittest.TestCase):
		def testEnterBinmode(self):
			bp = BusPirate(device)
		
		def testShortSelfTest(self):
			bp = BusPirate(device)
			self.assertTrue(bp.selfTest())
			
		def testEnterInvalidMode(self):
			bp = BusPirate(device)
			
			self.assertRaises(BusPirateError,bp.enterMode,'foo')
			
		def testUartMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode('uart'))
			
			self.assertTrue(bp.uartSetPins(power=1))
			self.assertTrue(bp.uartSetSpeed(300))
			self.assertTrue(bp.uartSetSpeed(9600))
			self.assertTrue(bp.uartSetConfig(output=1,parity='e'))
			self.assertTrue(bp.uartSetConfig())
			
			self.assertTrue(bp.uartSetPins())
			self.assertTrue(bp.leaveMode())
			
		def testSpiMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("SPI"))
			
			self.assertTrue(bp.leaveMode())

		def testI2CMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("I2C"))

			self.assertTrue(bp.leaveMode())

		def test1WireMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("1WIRE"))

			self.assertTrue(bp.leaveMode())

		def testRawMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("RAW"))

			self.assertTrue(bp.leaveMode())


		def testPWM(self):
			bp = BusPirate(device)
			self.assertTrue(bp.setPWM(0.5,2500))
			self.assertTrue(bp.stopPWM())

		
			
	import re
	import os
	suiteMock = None
	testloader = unittest.TestLoader()
#	suiteMock = testloader.loadTestsFromTestCase(BPTestMock)

	files = os.listdir("/dev")
	usbdevices = []
	for f in files:
		if f.startswith("tty.usbserial"):
			usbdevices.append("/dev/"+f)
			
	if len(usbdevices) > 0:
		suiteReal = testloader.loadTestsFromTestCase(BPTestReal)
		device = usbdevices[0]
		if len(usbdevices) > 1:
			print "Warning: multiple usbserial devices found, selecting %s" % device

	else:
		suiteReal=None
		print "BusPirate not found, disabling real device tests"
	
	if suiteMock:
		unittest.TextTestRunner(verbosity=2).run(suiteMock)
	if suiteReal:
		unittest.TextTestRunner(verbosity=2).run(suiteReal)
	
#	unittest.main()