#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
© Copyright 2015-2016, 3D Robotics.

my_vehicle.py:

Custom Vehicle subclass to add IMU data.
"""

from dronekit import Vehicle


class RawIMU(object):
	"""
	The RAW IMU readings for the usual 9DOF sensor setup. 
	This contains the true raw values without any scaling to allow data capture and system debugging.
	
	The message definition is here: https://pixhawk.ethz.ch/mavlink/#RAW_IMU
	
	:param time_boot_us: Timestamp (microseconds since system boot). #Note, not milliseconds as per spec
	:param xacc: X acceleration (mg)
	:param yacc: Y acceleration (mg)
	:param zacc: Z acceleration (mg)
	:param xgyro: Angular speed around X axis (millirad /sec)
	:param ygyro: Angular speed around Y axis (millirad /sec)
	:param zgyro: Angular speed around Z axis (millirad /sec)
	:param xmag: X Magnetic field (milli tesla)
	:param ymag: Y Magnetic field (milli tesla)
	:param zmag: Z Magnetic field (milli tesla)    
	"""
	def __init__(self, time_boot_us=None, xacc=None, yacc=None, zacc=None, xygro=None, ygyro=None, zgyro=None, xmag=None, ymag=None, zmag=None):
		"""
		RawIMU object constructor.
		"""
		self.time_boot_us = time_boot_us
		self.xacc = xacc
		self.yacc = yacc
		self.zacc = zacc
		self.xgyro = zgyro
		self.ygyro = ygyro
		self.zgyro = zgyro
		self.xmag = xmag        
		self.ymag = ymag
		self.zmag = zmag      
		
	def __str__(self):
		"""
		String representation used to print the RawIMU object. 
		"""
		return "RAW_IMU: time_boot_us={},xacc={},yacc={},zacc={},xgyro={},ygyro={},zgyro={},xmag={},ymag={},zmag={}".format(self.time_boot_us, self.xacc, self.yacc,self.zacc,self.xgyro,self.ygyro,self.zgyro,self.xmag,self.ymag,self.zmag)
	def display(self):
		return "{0},{1},{2}".format(self.xacc,self.yacc,self.zacc)
class Servo_Output(object):
	def __init__(self,servo1=None,servo2=None,servo3=None,servo4=None,servo5=None,servo6=None,servo7=None,servo8=None): 
		self.servo1=servo1
		self.servo2=servo2
		self.servo3=servo3
		self.servo4=servo4
		self.servo5=servo5
		self.servo6=servo6
		self.servo7=servo7
		self.servo8=servo8
	def __str__(self):
		return "SERVO_INPUT:servo1={},servo2={},servo3={},servo4={},servo5={},servo6={},servo7={},servo8={}".format(self.servo1,self.servo2,self.servo3,self.servo4,self.servo5,self.servo6,self.servo7,self.servo8)
	def display(self):
		return "{0},{1},{2},{3},{4},{5},{6},{7}".format(self.servo1,self.servo2,self.servo3,self.servo4,self.servo5,self.servo6,self.servo7,self.servo8)
class MyVehicle(Vehicle):
	def __init__(self, *args):
		super(MyVehicle, self).__init__(*args)

		# Create an Vehicle.raw_imu object with initial values set to None.
		self._raw_imu = RawIMU()
		self._servo_output=Servo_Output()
		# Create a message listener using the decorator.   
		@self.on_message('RAW_IMU')
		def listener(self, name, message):
			"""
			The listener is called for messages that contain the string specified in the decorator,
			passing the vehicle, message name, and the message.
			
			The listener writes the message to the (newly attached) ``vehicle.raw_imu`` object 
			and notifies observers.
			"""
			self._raw_imu.time_boot_us=message.time_usec
			self._raw_imu.xacc=message.xacc
			self._raw_imu.yacc=message.yacc
			self._raw_imu.zacc=message.zacc
			self._raw_imu.xgyro=message.xgyro
			self._raw_imu.ygyro=message.ygyro
			self._raw_imu.zgyro=message.zgyro
			self._raw_imu.xmag=message.xmag
			self._raw_imu.ymag=message.ymag
			self._raw_imu.zmag=message.zmag
			
			# Notify all observers of new message (with new value)
			#   Note that argument `cache=False` by default so listeners
			#   are updated with every new message
			self.notify_attribute_listeners('raw_imu', self._raw_imu) 
		@self.on_message('SERVO_OUTPUT_RAW')
		def listener(self, name, message):
			self._servo_output.servo1=message.servo1_raw
			self._servo_output.servo2=message.servo2_raw
			self._servo_output.servo3=message.servo3_raw
			self._servo_output.servo4=message.servo4_raw
			self._servo_output.servo5=message.servo5_raw
			self._servo_output.servo6=message.servo6_raw
			self._servo_output.servo7=message.servo7_raw
			self._servo_output.servo8=message.servo8_raw
					
			self.notify_attribute_listeners('servo_output', self._servo_output) 

	@property
	def raw_imu(self):
		return self._raw_imu
	@property
	def servo_output(self):
		return self._servo_output
