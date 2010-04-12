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
import struct


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
	UART_ENTER_BRIDGE = chr(0b1111)
	
	#tests
	TEST_SHORT = chr(0b10000)
	TEST_LONG = chr(0b10001)
	TEST_EXIT = chr(0xff)
	#pwm
	PWM_SET = chr(0b10010)
	PWM_CLEAR = chr(0b10011)

	#pins
	CONFIG_PINS = chr(0b01000000)
	SET_PINS = chr(0b10000000)

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
	"""BusPirate interface class.
	"""
	
	def __init__(self, port,**kwargs):
		"""
			@param port: serial port to use
			@keyword baudrate: baudrate to use, defaults to 115200
			@raise BusPirateError: if cannot enter Binary mode
		"""
		super(BusPirate, self).__init__()

		#use dummy serial device in unittests
		self.unittestSerial = kwargs.pop('unittestserial',None)

		self.port = port
		self.baudrate = kwargs.pop('baudrate',115200)
		self.serial = None
		self.open()
	
	def open(self):
		"""Opens connection to BusPirate and enters binary mode. Closes existing connection if already open
			@raise BusPirateError: if cannot enter Binary mode
		"""
		self.close()
		self.serial = self.unittestSerial or serial.Serial(self.port,self.baudrate)
		self.serial.timeout = 2.0 #seconds
		self.lastresponse=None
		self.mode = None
		self._uartEcho = False
		
		self._enterBinaryMode()
		
	def close(self):
		"""Resets BusPirate and closes serial connection"""
		if self.serial:
			self._sendCmd(Commands.RESET_USER)
			self.serial.close()
			self.serial=None
	
	def selfTest(self,longTest=False):
		"""Perform self-test.
		
			@param longTest: Perform long self-test. Requires jumpers between +5 and Vpu, +3.3 and ADC. Defaults to False
			@return: True or False
			@rtype: bool
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
		"""Enter protocol mode.
	
			@param mode: "spi","uart","i2c","1wire","raw"
			@raise BusPirateError: if command fails
			@raise ValueError: if parameters are invalid
			
		"""
		if self.mode:
			self.leaveMode()
			
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
				raise ValueError("Invalid mode '%s'" % mode)

			cmd,response = pair
			self._sendCmd(cmd,response)
			self.mode = mode
			
		except CommandError as err:
			raise BusPirateError(err)

		return True
		
	def leaveMode(self):
		"""Leave protocol mode."""
	
		if not self.mode:
			raise BusPirateError("Not in protocol mode")
			
		self._sendCmd(*CommandResponsePairs.RESET_BINARY)
		self.mode = None
		return True
	
	#voltage probe
	def probeVoltage(self):
		"""Perform AUX pin voltage measurement.
			@return: measured voltage.
			@rtype: float
		"""
		self._write(Commands.PROBE_VOLTAGE)
		data = self._read(2)
		
		v, = struct.unpack('>H',data)
		
		return (float(v)/1024)*6.6
		
	def configPins(self,**kwargs):
		"""Configure AUX,MOSI,CLK,MISO and CS as inputs (1) or outputs (0).
		
			@keyword aux: defaults to 1
			@keyword mosi: defaults to 1
			@keyword clk: defaults to 1
			@keyword miso: defaults to 1
			@keyword cs: defauls to 1
		
			@return: pin directions after update
			@rtype: int
		"""
		
		aux = kwargs.pop('aux',1)
		mosi = kwargs.pop('mosi',1)
		miso = kwargs.pop('miso',1)
		clk = kwargs.pop('clk',1)
		cs = kwargs.pop('cs',1)

		pins = (aux<<4) | (mosi<<3) | (clk<<2) | (miso<<1) | cs
		self._write(chr(ord(Commands.CONFIG_PINS)|pins))
		
		pins = self._read(1)
		return ord(pins)
	
	def setPins(self,**kwargs):
		"""Set pins POWER,AUX,MOSI,CLK,MISO,CS and PULLUP on (1) or off (0).
				
				@keyword power: defaults to 0
				@keyword pullup: defaults to 0
				@keyword aux: defaults to 0
				@keyword mosi: defaults to 0
				@keyword clk: defaults to 0
				@keyword miso: defaults to 0
				@keyword cs: defauls to 0
				
				@return: pin state after update
				@rtype: int
		"""
		power = kwargs.pop("power",0)
		aux = kwargs.pop('aux',0)
		mosi = kwargs.pop('mosi',0)
		miso = kwargs.pop('miso',0)
		clk = kwargs.pop('clk',0)
		cs = kwargs.pop('cs',0)
		pullup = kwargs.pop('pullup',0)
		
		pins = (power<<6)| (pullup<<5)| (aux<<4) | (mosi<<3) | (clk<<2) | (miso<<1) | cs
		self._write(chr(ord(Commands.SET_PINS)|pins))
		
		pins = self._read(1)
		return ord(pins)
	
	#pwm
	def setPWM(self,dutycycle,hz):
		"""Set AUX pin PWM
			@keyword duty: dutycycle (0.0 - 1.0)
			@keyword hz: pwm frequency
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
		"""Stop outputting PWM
			@raise BusPirateError: if command fails.
		"""
		self._sendCmd(*CommandResponsePairs.PWM_CLEAR)
		return True
	
	#{	UART functions
	def uartEnter(self):
		return self.enterMode("uart")
	
	def uartStartEcho(self):
		"""Start uart echo mode. Incoming data can be inspected with uartBytesAvailable and uartReceive
			@raise BusPirateError: if command fails.
		
		"""
		self._checkMode(Modes.UART)
		self._sendCmd(*CommandResponsePairs.UART_START_ECHO)
		self._uartEcho = True
		return True
		
	def uartStopEcho(self):
		"""Stop uart echo mode.
			@raise BusPirateError: if command fails.
		"""
		self._checkMode(Modes.UART)
		self._sendCmd(*CommandResponsePairs.UART_STOP_ECHO)
		return True
		
	def uartWrite(self,data):
		"""Writes data to uart
			@raise BusPirateError: if command fails.
		
		"""
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
		"""Returns number of bytes avaiable in uart echo mode.
			
			@raise BusPirateError: if command fails.
		"""
		self._checkMode(Modes.UART)
		if not self._uartEcho:
			raise BusPirateError("Not in UART echo mode")

		return self._available()
		
	def uartReceive(self,size=1):
		"""Receive bytes in uart echo mode.
			@param size: number of bytes to read
			@raise BusPirateError: if command fails.
		"""
		self._checkMode(Modes.UART)
		if not self._uartEcho:
			raise BusPirateError("Not in UART echo mode")

		return self._read(size)
		
	def uartSetSpeed(self,baudrate=9600):
		"""configures uart baudrate
			@param baudrate: 300,1200,2400,4800,9600,19200,31250,28400,57600,115200
			@raise BusPirateError: if command fails.
			@raise ValueError: if parameter is invalid
		"""
		self._checkMode(Modes.UART)

		rates = { 300 : 0,  1200 : 0b0001, 2400: 0b0010,4800 : 0b0011,9600 : 0b0100,19200: 0b0101,31250 : 0b0110, 38400 : 0b0111,57600 : 0b1000,115200 : 0b1010}
		
		if baudrate not in rates:
			raise ValueError("Invalid baudrate '%d'" % baudrate)
		
		self._sendCmd(chr(ord(Commands.UART_SET_BAUDRATE)|rates[baudrate]),Responses.OK)
		return True
		
	def uartSetPins(self,**kwargs):
		"""Configure peripherals.
			@keyword power: defaults to 0
			@keyword pullups: defaults to 0
			@keyword aux: defaults to 0
			@keyword cs: defaults to 0
			@raise BusPirateError: if command fails.
			
		"""
		self._checkMode(Modes.UART)
		
		return self._setPins(**kwargs)
		
	def uartSetConfig(self,**kwargs):
		"""Set UART configuration.
			@keyword output: 0 = HiZ, 1 = 3.3v, defaults to HiZ (0)
			@keyword databits: 8 or 9, defaults to 8
			@keyword parity: 'N' (none) or 'E' (even) or 'O' (odd), defaults to 'N'
			@keyword stopbits: stop bits , defaults to 1
			@keyword polarity: 0 = idle high, 1 = idle low, defaults to idle high (0)
			@raise BusPirateError: if command fails.
			@raise ValueError: if parameters are invalid.
		"""
		self._checkMode(Modes.UART)
		
		output = kwargs.pop("output",0)
		databits = kwargs.pop("databits",8)
		parity = kwargs.pop('parity','N').lower()
		stopbits = kwargs.pop("stopbits",1)
		polarity = kwargs.pop("polarity",0)
		
		#verify parameters
		if output != 0 and output != 1:
			raise ValueError("Output value is invalid")
		if stopbits != 0 and stopbits != 1:
			raise ValueError("Stopbits value is invalid")
		if polarity != 0 and polarity != 1:
			raise ValueError("Polarity is invalid")
		
		#combine databits and parity
		dp = 0
		if parity not in 'eno':
			raise ValueError("Parity is invalid")
			
		if databits == 9:
			if parity == 'e':
				raise ValueError("Parity cannot be even if databits is 9")
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
			raise ValueError("Databits value is invalid.")
		
		self._sendCmd(chr(ord(Commands.UART_SET_CONFIG) | (output<<4) | (dp << 3) | (stopbits << 1) | polarity),Responses.OK)
		return True
		
	def uartBridgeMode(self):
		"""Enters UART bridge mode. NOTE: Bridge mode cannot be exited programmatically, Bus Pirate has to be reseted manually.

			@return: a pyserial instance which can be used directly
		"""
		self._checkMode(Modes.UART)
		self._sendCmd(Commands.UART_ENTER_BRIDGE)
		return self.serial
	#}	
		
	#{ I2C functions
	def i2cEnter(self):
		return self.enterMode("i2c")
		
	def i2cSendStart(self):
		self._checkMode(Modes.I2C)
		raise NotImplementedError
		
	def i2cSendStop(self):
		self._checkMode(Modes.I2C)

		raise NotImplementedError
		
	def i2cSendAck(self):
		self._checkMode(Modes.I2C)

		raise NotImplementedError
	def i2cSendNack(self):
		self._checkMode(Modes.I2C)

		raise NotImplementedError
		
	def i2cReadByte(self):
		self._checkMode(Modes.I2C)

		raise NotImplementedError
	
	def i2cStartSniffer(self):
		self._checkMode(Modes.I2C)

		raise NotImplementedError
		
	def i2cWrite(self,data):
		self._checkMode(Modes.I2C)

		raise NotImplementedError
		
	def i2cSetPins(self,**kwargs):
		"""Configure peripherals.
			@keyword power: defaults to 0
			@keyword pullups: defaults to 0
			@keyword aux: defaults to 0
			@keyword cs: defaults to 0
		"""

		self._checkMode(Modes.I2C)

		return self._setPins(**kwargs)
		
	def i2cSetSpeed(self,speed=400):
		self._checkMode(Modes.I2C)

		raise NotImplementedError
	
	#}	
	#{ SPI functions
	def spiEnter(self):
		return self.enterMode("spi")
		
	def spiStartSniffer(self):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiSetSnifferMode(self,mode):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiWrite(self,data):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiWriteByte(self,data):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiSetSpeed(self,speed=30):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiGetSpeed(self):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiSetPins(self,**kwargs):
		"""Configure peripherals.
			@keyword power: defaults to 0
			@keyword pullups: defaults to 0
			@keyword aux: defaults to 0
			@keyword cs: defaults to 0
		"""

		self._checkMode(Modes.SPI)

		return self._setPins(**kwargs)
	
	def spiGetPins(self):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiSetConfig(self,**kwargs):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
		
	def spiGetConfig(self):
		self._checkMode(Modes.SPI)

		raise NotImplementedError
	#}	
		
	#{ 1-wire functions
	def onewireEnter(self):
		self._checkMode(Modes.ONEWIRE)

		return self.enterMode("1wire")
		
	def onewireReset(self):
		self._checkMode(Modes.ONEWIRE)

		raise NotImplementedError
		
	def onewireReadByte(self):
		self._checkMode(Modes.ONEWIRE)

		raise NotImplementedError
		
	def onewireSearchRom(self):
		self._checkMode(Modes.ONEWIRE)

		raise NotImplementedError
	
	def onewireSearchAlarm(self):
		self._checkMode(Modes.ONEWIRE)

		raise NotImplementedError
		
	def onewireWrite(self,data):
		self._checkMode(Modes.ONEWIRE)

		raise NotImplementedError
		
	def onewireSetPins(self,**kwargs):
		"""Configure peripherals.
			@keyword power: defaults to 0
			@keyword pullups: defaults to 0
			@keyword aux: defaults to 0
			@keyword cs: defaults to 0
		"""

		self._checkMode(Modes.ONEWIRE)

		return self._setPins(**kwargs)
	#}
	
	#{ RAW functions
	def rawEnter(self):
		return self.enterMode("raw")
		
	def rawSetCS(self,pin=0):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
		
	def rawReadByte(self):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
		
	def rawReadBit(self):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
		
	def rawPeekInput(self):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
		
	def rawClockTick(self,ticks=1):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
		
	def rawSetClock(self,pin=0):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
		
	def rawSetData(self,pin=0):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
		
	def rawWrite(self,data):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
	
	def rawSetPins(self,**kwargs):
		"""Configure peripherals.
			@keyword power: defaults to 0
			@keyword pullups: defaults to 0
			@keyword aux: defaults to 0
			@keyword cs: defaults to 0
		"""
		
		self._checkMode(Modes.RAW)

		return self._setPins(**kwargs)
		
	def rawSetSpeed(self,speed=5):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
	
	def rawSetConfig(self,**kwargs):
		self._checkMode(Modes.RAW)

		raise NotImplementedError
	#}
	
	#internal functions
	def _setPins(self,**kwargs):
		power = kwargs.pop("power",0)
		pullups = kwargs.pop("pullups",0)
		aux = kwargs.pop("aux",0)
		cs = kwargs.pop("cs",0)
		cmd = chr(ord(Commands.CONFIG_PINS) | (power << 3) | (pullups << 2) | (aux << 1) | cs)
		
		self._sendCmd(cmd,Responses.OK)
		return True
		
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
			ret = ""
			i=0
			while len(ret) < length and i < 10:
				ret += self.serial.read(length)
				i+=1
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

#unittests
if __name__ == '__main__':
	import unittest
	import re
	device = None

	#tests that use real device
	class BPTests(unittest.TestCase):
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
			self.assertRaises(BusPirateError,bp.uartSetConfig,output=1,parity='d')
			self.assertTrue(bp.uartSetConfig())
			
			self.assertTrue(bp.uartSetPins())
			self.assertTrue(bp.leaveMode())
			self.assertTrue(bp.uartEnter())
			
		def testSpiMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("SPI"))
			self.assertTrue(bp.spiEnter())
			
			self.assertTrue(bp.leaveMode())

		def testI2CMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("I2C"))
			self.assertTrue(bp.i2cEnter())

			self.assertTrue(bp.leaveMode())

		def test1WireMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("1WIRE"))
			self.assertTrue(bp.onewireEnter())

			self.assertTrue(bp.leaveMode())

		def testRawMode(self):
			bp = BusPirate(device)
			self.assertTrue(bp.enterMode("RAW"))
			self.assertTrue(bp.rawEnter())

			self.assertTrue(bp.leaveMode())

		def testProbeVoltage(self):
			bp = BusPirate(device)
			self.assertEqual(0.0,bp.probeVoltage())

		def testPWM(self):
			bp = BusPirate(device)
			self.assertTrue(bp.setPWM(0.5,2500))
			self.assertTrue(bp.stopPWM())

		def testConfigPins(self):
			bp = BusPirate(device)
			self.assertTrue(0b01010100,bp.configPins(aux=1,clk=1))
			self.assertTrue(0b01000000,bp.configPins())

		def testSetPins(self):
			bp = BusPirate(device)
			self.assertTrue(0b1110000,bp.setPins(power=1,pullup=1))
			self.assertTrue(0b1000000,bp.setPins())



			
	import re
	import os

	files = os.listdir("/dev")
	usbdevices = []
	for f in files:
		if f.startswith("tty.usbserial"):
			usbdevices.append("/dev/"+f)
			
	if len(usbdevices) > 0:

		device = usbdevices[0]
		if len(usbdevices) > 1:
			print "Warning: multiple usbserial devices found, selecting %s" % device

		suite = unittest.TestLoader().loadTestsFromTestCase(BPTests)
		unittest.TextTestRunner(verbosity=2).run(suite)
	else:
		print "BusPirate not found, cannot run tests"

		
#	unittest.main()