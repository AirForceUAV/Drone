#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time,math,json
from pymavlink import mavutil


__college__='PKU'
__author__ ='mengxz'
__BeginningDate__   ='2016/4/13'
__EndingDate__='2016/5/22'

		
class Drone(object):
	def __init__(self,args='sitl'):
		self.gps_lock = False
		self.altitude = 5.0
		self.vehicle = self.connection(args)
		#self.commands = self.vehicle.commands
		self.home_location = None
		self.current_location=None
		self.sitl=None
		self.target=None
		
	def connection(self,args):
		args=str(args)
		if args is 'sitl' or args is None:
			self._log("Starting copter simulator (SITL)")
			from dronekit_sitl import SITL
			sitl = SITL()
			self.sitl=sitl
			sitl.download('copter', '3.3', verbose=True)
			sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,0']
			sitl.launch(sitl_args, await_ready=True, restart=True)
			connection_string='tcp:127.0.0.1:5760'
		elif args=='0':
			connection_string='/dev/ttyACM0'
		elif args=='1': 
			connection_string='/dev/ttyACM1'
		else:
			connection_string=str(args)
		self._log('Connecting to vehicle on: %s' % connection_string)
		vehicle = connect(connection_string, wait_ready=True)

		# Register observers
		vehicle.add_attribute_listener('location',self.location_callback)
		vehicle.add_attribute_listener('battery',self.battery_callback)
		#vehicle.add_attribute_listener('heading',self.heading_callback)
		return vehicle
	def location_callback(self, vehicle, name, location):
		if location.global_relative_frame.alt is not None:
			self.altitude = location.global_relative_frame.alt

		self.current_location = location.global_relative_frame
	def battery_callback(self,vehicle, name,battery):
		if battery.level<20:
			self._log('Alert ,Low Battery! Remaining Power: {0} '.format(battery.level))
	def heading_callback(self,vehicle,name,heading):
		self._log('Current heading: {0}'.format(heading))
	def get_vehicle(self):
		return self.vehicle
	def get_alt(self,relative=True):
		if relative==True:
			return self.vehicle.location.global_relative_frame.alt
		else:
			return self.vehicle.location.global_frame.alt
	def get_bearing(self,aLocation1, aLocation2):
		"""
		Returns the bearing between the two LocationGlobal objects passed as parameters.

		This method is an approximation, and may not be accurate over large distances and close to the 
		earth's poles.
		"""	
		off_x = aLocation2.lon - aLocation1.lon
		off_y = aLocation2.lat - aLocation1.lat
		bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
		if bearing < 0:
			bearing += 360.00
		return bearing

	def get_angle_north(self,lat,lon):
		'''Returns the bearing between currentLocation and Given lat/lon''' 
		aLoction1=self.get_location()
		aLocation2=LocationGlobal(lat,lon)
		angle_North=self.get_bearing(aLoction1,aLocation2)		
		return int(angle_North)

	def get_angle_heading(self):
		target=self.get_target()
		if target is None:
			self._log("Target is None! ")
			return None
		angle_North=self.get_angle_north(target.lat,target.lon)
		angle_heading=angle_North-self.get_heading()
		if angle_heading<0:
			angle_heading+=360	
		return int(angle_heading)
	def get_parameters(self):
		self._log("\nPrint all parameters (iterate `vehicle.parameters`):")
		params={}
		for key, value in self.vehicle.parameters.iteritems():
			params[key]=value
		return json.dumps(params)
	def get_parameter(self,key):
		return self.vehicle.parameters[key]
	def set_parameter(self,key,value):
		self.vehicle.parameters[key]=value
	
	def get_target(self):		
		return self.target
	def set_airspeed(self,airspeed):
		self.vehicle.airspeed=airspeed

	def set_groundspeed(self,groundspeed):
		self.vehicle.groundspeed=groundspeed

	def arm(self):
		
		# Get Vehicle Home location - will be `None` until first set by autopilot
		self._log("Waiting for home location...")
		while not self.vehicle.home_location:
			cmds = self.vehicle.commands
			cmds.download()
			cmds.wait_ready()
			time.sleep(.1)
		self._log("\n Home location: {0}" .format(self.vehicle.home_location))
		#Get home_location
		
		while self.vehicle.location.global_frame.lat==0:
			time.sleep(.1)
		self.home_location = self.get_location()
		#self._log('\n Home is:{0}'.format(self.get_home()))
		#Set GUIDED Mode
		self._log("Set Vehicle.mode = GUIDED (currently: {0})".format(self.vehicle.mode.name) ) 
		self.vehicle.mode = VehicleMode("GUIDED")
		self._log("Waiting for mode change ...")
		while not self.vehicle.mode.name=='GUIDED':  #Wait until mode has changed			
			time.sleep(.1)
		self._log('current mode :{0}'.format(self.vehicle.mode.name))
		#Check is_armable
		self._log("Waiting for ability to arm...")
		while not self.vehicle.is_armable:
			time.sleep(.1)

		#Armed
		self.vehicle.armed = True
		self._log("Waiting for arming...")
		while not self.vehicle.armed:      			
			time.sleep(.1)
	
	def disarm(self):
		self._log("DisArming")
		self.vehicle.armed=False
		
	def takeoff(self,alt=5):
		self._log("Taking off to {0}m".format(alt))
		self.vehicle.simple_takeoff(alt)
		# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command after Vehicle.simple_takeoff will execute immediately).
		while True:
			self._log("Altitude: {0}".format(self.vehicle.location.global_relative_frame.alt))      
			if self.vehicle.location.global_relative_frame.alt>=alt*0.95: #Trigger just below target alt.
				self._log("Reached target altitude")
				break
			time.sleep(1)
  
	def get_home(self):
		return self.home_location
		
	def get_location(self):
		#self._log('Current Location:{0}'.format(self.vehicle.location.global_relative_frame))
		return self.vehicle.location.global_relative_frame

	def is_stable(self):
		return self.vehicle.version.release==255

	def release_type(self):
		if self.vehicle.version.release is None:
			return None
		types = [ "dev", "alpha", "beta", "rc" ]
		return types[self.vehicle.version.release/64]

	def version(self):
		prefix=""

		if(self.vehicle._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
			prefix += "APM:"
		elif(self.vehicle._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4):
			prefix += "PX4"
		else:
			prefix += "UnknownAutoPilot"

		if(self.vehicle._vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR):
			prefix += "Copter-"
		elif(self.vehicle._vehicle_type == mavutil.mavlink.MAV_TYPE_FIXED_WING):
			prefix += "Plane-"
		elif(self.vehicle._vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR):
			prefix += "Hexa-"
		elif(self.vehicle._vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER):
			prefix += "Rover-"
		else:
			prefix += "UnknownVehicleType%d-" % (self.vehicle._vehicle_type)

		if self.release_type() is None:
			release_type = "UnknownReleaseType"
		elif self.is_stable():
			release_type = ""
		else:
			# e.g. "-rc23"
			release_type = "-" + str(self.release_type()) + str(self.release_version())

		return prefix + "%s.%s.%s" % (self.vehicle.version.major, self.vehicle.version.minor, self.vehicle.version.patch) + release_type

	def FC_info(self):
		'''Return FlightControl information'''
		vehicle=self.vehicle
		
		c=vehicle.capabilities
		version=self.version()

		obj={
			'Version':version,
			'Capabilities':(c.mission_float,c.param_float,c.mission_int,c.command_int,c.param_union,c.ftp,c.set_attitude_target,
			 c.set_attitude_target_local_ned,c.set_altitude_target_global_int,c.terrain,c.set_actuator_target,c.flight_termination,c.compass_calibration),
			}
				
		encodedjson=json.dumps(obj)
		return encodedjson

	def UAV_info(self):
		vehicle=self.vehicle
		obj={'LocationGlobal':(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt),
			'Local_location':(vehicle.location.local_frame.north,vehicle.location.local_frame.east,vehicle.location.local_frame.down),
			'Attitude':(vehicle.attitude.pitch,vehicle.attitude.yaw,vehicle.attitude.roll),
			'Velocity':vehicle.velocity,
			'GPS':(vehicle.gps_0.eph,vehicle.gps_0.epv,vehicle.gps_0.fix_type,vehicle.gps_0.satellites_visible),
			'Gimbal_status':(vehicle.gimbal.pitch,vehicle.gimbal.yaw,vehicle.gimbal.roll),
			'Battery':(vehicle.battery.voltage,vehicle.battery.current,vehicle.battery.level),
			'EKF':vehicle.ekf_ok,
			'LastHeartbeat':vehicle.last_heartbeat,
			'Rangefinder':(vehicle.rangefinder.distance,vehicle.rangefinder.voltage),
			'Heading':vehicle.heading,
			'Groundspeed':vehicle.groundspeed,
			'Airspeed':vehicle.airspeed,
			'System_status':vehicle.system_status.state,
			'Mode':vehicle.mode.name
			}
		encodedjson=json.dumps(obj)
		return encodedjson

	def goto_NED(self,dNorth,dEast):
		currentLocation=self.vehicle.location.global_relative_frame
		targetLocation=self.get_location_metres(currentLocation, dNorth, dEast)
		targetDistance=self.get_distance_metres(currentLocation, targetLocation)
		self.vehicle.simple_goto(targetLocation)

		while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
			remainingDistance=self.get_distance_metres(self.vehicle.location.global_frame, targetLocation)
			print "Distance to target: ", remainingDistance
			if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
				print "Reached target"
				break
			time.sleep(2)
	def goto(self, lat,lon):
		self._log("Goto: {0},{1} ,{2}".format(lat,lon, self.altitude))
		
		self.vehicle.simple_goto(LocationGlobalRelative(float(lat), float(lon),float(self.get_alt())))
		self.vehicle.flush()

	def report_remainingDistance(self,currentlocation,targetLocation):
		targetDistance = self.get_distance_metres(currentlocation, targetLocation)   #get distance between currentlocation and targetlocation
		while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
			remainingDistance=self.get_distance_metres(self.get_location(), targetLocation)
			self._log("Distance to target:{0}, heading:{1} ".format(remainingDistance,self.get_heading()))
			if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
				self._log("Reached target")
				break;
			time.sleep(2)
	def condition_yaw(self,heading, relative=True):
		'''After taking off, yaw commands are ignored until the first “movement” command has been received. 
		If you need to yaw immediately following takeoff then send a command to “move” to your current position'''
		if heading<0:
			self._log("Turn Left {0}".format(-heading))
		elif heading>0:
			self._log("Turn Right {0}".format(heading))

		if relative:
			is_relative = 1 #yaw relative to direction of travel
		else:
			is_relative = 0 #yaw is an absolute angle
		if heading>=0:
			is_cw=1
		else:
			is_cw=-1
			heading=-heading

		# create the CONDITION_YAW command using command_long_encode()

		msg =self. vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		is_cw,          # param 3, direction -1 ccw, 1 cw
		is_relative, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
		# send command to vehicle
		self. vehicle.send_mavlink(msg)
		

	def go_home(self,alt=None):
		'''Default alt is current altitude'''
		
		if alt==None:
			alt=self.get_alt()
		lat=self.get_home().lat
		lon=self.get_home().lon
		self._log('Go Home!lat:{0} lon:{1} alt: {2}'.format(lat,lon,alt))
		aLocation=LocationGlobalRelative(lat,lon,alt)
		self.vehicle.simple_goto(aLocation)

	def set_roi(self,location):
		# create the MAV_CMD_DO_SET_ROI command
		msg = self.vehicle.message_factory.command_long_encode(
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
			0, #confirmation
			0, 0, 0, 0, #params 1-4
			location.lat,
			location.lon,
			location.alt
			)
		# send command to vehicle
		self.vehicle.send_mavlink(msg)

	def send_global_velocity(self,velocity_x, velocity_y, velocity_z, duration):
		msg = self.vehicle.message_factory.set_position_target_global_int_encode(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
		0b0000111111000111, # type_mask (only speeds enabled)
		0, # lat_int - X Position in WGS84 frame in 1e7 * meters
		0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
		# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
		velocity_x, # X velocity in NED frame in m/s
		velocity_y, # Y velocity in NED frame in m/s
		velocity_z, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_
		for x in range(0,int(duration)):
			self.vehicle.send_mavlink(msg)
			time.sleep(1)   

	def send_body_offset_ned_velocity(self,forward,right,down,duration):
		msg=self.vehicle.message_factory.set_position_target_local_ned_encode(
		0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0,0,0,
		forward,right,down,    #vx,vy,vz
		0,0,0,   #afx,afy,afz
		0,0)    #yaw yaw_rate
		for x in range(0,int(duration)):
			self.vehicle.send_mavlink(msg)
			time.sleep(1)
	def send_body_offset_ned_position(self,forward=0,right=0,down=0):
		msg=self.vehicle.message_factory.set_position_target_local_ned_encode(
		0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111111000,
		forward,right,down,
		0,0,0,    #vx,vy,vz
		0,0,0,   #afx,afy,afz
		0,0)    #yaw yaw_rate
		self.vehicle.send_mavlink(msg)

	#control movement by position
	def forward_p(self,distance=0.5):
		self.send_body_offset_ned_position(distance,0,0)
	def backward_p(self,distance=0.5):
		self.send_body_offset_ned_position(-distance,0,0)
	def left_p(self,distance=0.5):
		self.send_body_offset_ned_position(0,-distance,0)
	def right_p(self,distance=0.5):
		self.send_body_offset_ned_position(0,distance,0)
	def up_p(self,distance=0.5):
		self.send_body_offset_ned_position(0,0,-distance)
	def down_p(self,distance=0.5):
		self.send_body_offset_ned_position(0,0,distance)

	#control movemnet by velocity
	def forward(self,distance=0.5,velocity=1.0):
		self._log("Forward to {0}m,velocity is {1}m/s".format(distance,velocity))
		duration=distance/velocity
		if duration==0:
			duration=1
		self.send_body_offset_ned_velocity(velocity,0,0,duration)
		self.stop()
	def backward(self,distance=0.5,velocity=1.0):
		self._log("Backward to {0}m,velocity is {1}m/s".format(distance,velocity))
		duration=distance/velocity
		if duration==0:
			duration=1
		self.send_body_offset_ned_velocity(-velocity,0,0,duration)
		self.stop()
	def left(self,distance=0.5,velocity=1.0):
		self._log("Left to {0}m,velocity is {1}m/s".format(distance,velocity))
		duration=distance/velocity
		if duration==0:
			duration=1
		self.send_body_offset_ned_velocity(0,-velocity,0,duration)
		self.stop()
	def right(self,distance=0.5,velocity=1.0):
		self._log("Right to {0}m,velocity is {1}m/s".format(distance,velocity))
		duration=distance/velocity
		if duration==0:
			duration=1
		self.send_body_offset_ned_velocity(0,velocity,0,duration)
		self.stop()
	def yaw_left(self,angle=3):
		self.condition_yaw(-angle)
	def yaw_right(self,angle=3):
		self.condition_yaw(angle)
	def up(self,distance=0.5,velocity=1.0):
		self._log("Up to {0}m,velocity is {1}m/s".format(distance,velocity))
		duration=distance/velocity
		if duration==0:
			duration=1
		self.send_body_offset_ned_velocity(0,0,-velocity,duration)
		self.stop()
	def down(self,distance=0.5,velocity=1.0):
		self._log("Down to {0}m,velocity is {1}m/s".format(distance,velocity))
		duration=distance/velocity
		if duration==0:
			duration=1
		self.send_body_offset_ned_velocity(0,0,velocity,duration)
		self.stop()
	def stop(self):
		self.send_body_offset_ned_velocity(0,0,0,1)

	def fly(self,distance,heading=0,velocity=1.0):
		if heading is not 0:
			self.condition_yaw(heading)
		self.forward(distance,velocity)
		self._log('Reached')

	def fly_position(self,distance,heading=0):
		currentlocation=self.get_location()		
		targetlocation =self.get_location_distance_heading(currentlocation,distance,heading)
		self.condition_yaw(heading)
		
		self.forward_p(distance)		
		self.report_remainingDistance(currentlocation,targetlocation)
	
	def get_heading(self):
		#self._log('Current  heading:{0}'.format(self.vehicle.heading))
		return self.vehicle.heading
		
	def get_location_metres(self,original_location, dNorth, dEast):
		"""
		Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
		specified `original_location`. The returned LocationGlobal has the same `alt` value
		as `original_location`.

		The function is useful when you want to move the vehicle around specifying locations relative to 
		the current vehicle position.

		The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles."""

		earth_radius = 6378137.0 #Radius of "spherical" earth
		#Coordinate offsets in radians
		dLat = dNorth/earth_radius
		dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

		#New position in decimal degrees
		newlat = original_location.lat + (dLat * 180/math.pi)
		newlon = original_location.lon + (dLon * 180/math.pi)
		if type(original_location) is LocationGlobal:
			targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
		elif type(original_location) is LocationGlobalRelative:
			targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
		else:
			raise Exception("Invalid Location object passed")
			
		return targetlocation
	
	def set_target_metres(self,dNorth,dEast):
		location=self.get_location_metres(self.get_location(),dNorth,dEast)
		self.set_target(location.lat,location.lon)

	def set_target(self,lat,lon):
		if lat is None or lon is None:
			self._log("lat or lon can not be empty!")
		else:
			self.target = LocationGlobalRelative(lat,lon,self.get_alt())
			self._log("New target is {0}".format(self.target))

	def get_distance_metres(self,aLocation1, aLocation2):  
		#print 'a1:{0},a2:{1}'.format(aLocation1,aLocation2)     
		dlat = aLocation2.lat - aLocation1.lat
		dlong = aLocation2.lon - aLocation1.lon
		return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

	def get_location_distance_heading(self,currentlocation,distance,heading=0):
		radian=self.radian(heading)
		north=math.cos(radian)*distance
		east=math.sin(radian)*distance
		targetlocation= self.get_location_metres(currentlocation,north,east)
		#print "currentLocation:{0} , targetlocation:{1}".format(currentLocation,targetlocation)
		return targetlocation
	def radian(self,heading=0):
		angle=(self.get_heading()+heading)%360
		if angle<0:
			angle+=360
		radian=float(angle)/180*math.pi
		return radian
	def RTL(self):
		self.retract()
		self._log("Return To Home")
		self.vehicle.mode=VehicleMode("RTL")
	def Land(self):
		self.retract()
		self._log("Landing...")
		self.vehicle.mode=VehicleMode("LAND")
	def Stab(self):
		self._log("STABILIZE")
		self.vehicle.mode=VehicleMode("STABILIZE")
	def Loiter(self):
		self._log("Loiter")
		self.vehicle.mode=VehicleMode("LOITER")
	def Althold(self):
		self._log('Althold')
		self.vehicle.mode=VehicleMode("ALT_HOLD")

	def deploy(self): 
		'''LRG_SERVO_DEPLOY=1750''' 
		if self.vehicle.channels['8']<1750:  
			self._log('Deploy Landing Gear')
			self.vehicle.channels.overrides['8']=1900
			self._log('Waiting for deploying!')
			time.sleep(3)
		else:
			self._log('yet deployed')
	def retract(self):   
		'''LRG_SERVO_RTRACT=1250'''  
		if self.vehicle.channels['8']>1250:
			self._log('Retract Landing Gear')
			self.vehicle.channels.overrides['8']=1100
			self._log('Waiting for retracting')
			time.sleep(3)
		else:
			self._log('yet retracted')
	def get_mode(self):
		return self.vehicle.mode.name

	def get_channels(self):
		return self.vehicle.channels
	def get_channel(self,key):
		if not (int(key) > 0 and int(key) <=8):
			raise Exception('Invalid channel index %s' % key)
		return self.vehicle.channels[key]
	def set_channel(self,key,value):
		if not (int(key) > 0 and int(key) <=8):
			raise Exception('Invalid channel index %s' % key)
		self.vehicle.channels.overrides[key]=value
	def default_channel(self,key):
		if not (int(key) > 0 and int(key) <=8):
			raise Exception('Invalid channel index %s' % key)
		self.vehicle.channels.overrides[key]=None
	def default_channels(self):
		self.vehicle.channels.overrides={}
	
	def show(self):
		if self.get_home() is None:
			raise Exception('Home is None ,please have a try after arming')
		else:
			distance=self.get_distance_metres(self.get_home(),self.get_location())
			self._log('Distance to home:{0}m'.format(distance))
		location=self.get_location()
		#print "distance to home:{0},heading:{1},location:{2}".format(distance,self.get_heading(),self.get_location())
		self._log("heading:{0},lat:{1},lon:{2},alt:{3}".format(self.get_heading(),location.lat,location.lon,location.alt))

	def close(self):
		self._log("Close vehicle object")
		self.vehicle.close()
		# Shut down simulator if it was started.
		if self.sitl is not None:
			self._log('close SITL')
			self.sitl.stop()

	def __str__(self):
		helper="Please read README.md"
		return  helper
	def _log(self, message):
		print "[DEBUG]:"+message

if __name__=="__main__":
	drone=Drone()
	#print drone.FC_info()	
	#print drone.get_angle_north(-35.453261,149.245230)
	print drone.get_parameters()
	drone.arm()
	drone.takeoff(5)
	drone.set_channel(8,1100)
	drone.deploy()
	drone.stop()
	#drone.show()
	drone.condition_yaw(135,False)
	drone.fly(5)
	#drone.show()
	drone.fly(10,135)
	#drone.show()
	drone.fly(5,135)
	#drone.show()

	drone.go_home()
	#time.sleep(10)
	#drone.show()
	drone.Land()
	#time.sleep(10)
	#drone.show()


	drone.close()

	print("Completed")
	
	