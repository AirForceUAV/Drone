#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
from pymavlink import mavutil
import math 
__author__='mengxz'

class Ran(object):   
    __slots__=('sitl','vehicle')
    def __init__(self,link='SITL'):
        self.connect(link)
        #self.vehicle.add_attribute_listener('mode',self.mode_callback)
    def connect(self,link='SITL'):
        if link=='SITL':
            print "Starting copter simulator (SITL)"
            from dronekit_sitl import SITL
            sitl = SITL()
            self.sitl=sitl
            sitl.download('copter', '3.3', verbose=True)
            sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
            sitl.launch(sitl_args, await_ready=True, restart=True)
            connection_string = 'tcp:127.0.0.1:5760'
        else:
           connection_string =link
        print "\nConnecting to vehicle on:%s"%connection_string
        self.vehicle=connect(connection_string,wait_ready=True)
    
    def getVehicle(self):
       return self.vehicle
    def arm(self,mode='GUIDED'):
        print "Basic pre-arm checks"
        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print " Waiting for home location ..."
        # We have a home location, so print it!        
        print "\n Home location: %s" % self.vehicle.home_location
        # Check that vehicle is armable
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)
        print "Arming motors"
        self.setMode(mode)
        self.vehicle.armed = True    
        while not self.vehicle.armed:
            print('waiting for arming!')
            time.sleep(1)
    def disarm(self):
        if self.vehicle.armed==True:
            self.vehicle.armed=False;
    #print vehicle attribute values:
    def getVersion(self):
         print " Autopilot Firmware version: %s" % self.vehicle.version
    def getLocation(self):
        print self.vehicle.location.global_frame
        print self.vehicle.location.global_relative_frame
        print self.vehicle.location.local_frame
    def getAtt(self):
        print " Attitude: %s" % self.vehicle.attitude
    def getAlt(self):
        self.vehicle.location.global_relative_frame.alt 
    def getVel(self):
        print " Velocity: %s" % self.vehicle.velocity
    def getGPS(self):
        print " GPS: %s" % self.vehicle.gps_0
    def getMode(self):
       return self.vehicle.mode.name
    def getSpeed(self):
        print " Groundspeed: %s" % self.vehicle.groundspeed    # settable
        print " Airspeed: %s" % self.vehicle.airspeed    # settable
    def setMode(self,mode):
            self.vehicle.mode = VehicleMode(mode)
    def getHome(self):
        return self.vehicle.home_location
    def getAll(self):
        self.getGPS()
        self.getAtt()
        self.getVel()
        self.getSpeed()
        self.getLocation()
        print self.getHome()
        print "Heading:%s"%self.vehicle.heading
        print "ALT:%s"%self.getAlt()
        self.getMode()
    
    def take_off(self,aTargetAltitude=2):       
        #self.arm()
        print "Taking off!"
        self. vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt 
            #Break and return from function just below target altitude.        
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
                print "Reached target altitude"
                break
            time.sleep(1)
    
    def  mode_callback(self,attr_name,value):
        print "CALLBACK:mode changed to",value
    def condition_yaw(self,heading, relative=False):
         if relative:
             is_relative = 1 #yaw relative to direction of travel
         else:
            is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
         msg =self. vehicle.message_factory.command_long_encode(
         0, 0,    # target system, target component
         mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
         0, #confirmation
         heading,    # param 1, yaw in degrees
         0,          # param 2, yaw speed deg/s
         1,          # param 3, direction -1 ccw, 1 cw
         is_relative, # param 4, relative offset 1, absolute angle 0
         0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
         self. vehicle.send_mavlink(msg)
    def condition_yaw(self,heading, relative=True):
          if relative:
            is_relative = 1 #yaw relative to direction of travel
          else:
             is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
          msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
          self.vehicle.send_mavlink(msg)
    def send_global_velocity(self,velocity_x, velocity_y, velocity_z, duration):
            msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0, 
            0, 0, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
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
            for x in range(0,duration):
                self.vehicle.send_mavlink(msg)
                time.sleep(1)   
    def goto_by_yaw(self,heading=0,x=0.5,y=0.5,z=0,duration=5):
        currentHeading=self.vehicle.heading
        angle=currentHeading+heading
        if angle>=0 and angle<90:   #NorthEast
            pass
        elif angle>=90 and angle<180:    #SouthEast
            x=-x
        elif angle>=180 and angle<270:    #southWest
            x=-x
            y=-y
        elif angle>=270 and angle<360:       #NorthWest
            y=-y
        self.condition_yaw(heading)
        #print "pre-angle :%s"%self.vehicle.heading
        self.send_global_velocity(x,y,z,duration)
        self.send_global_velocity(0,0,0,1)
        #print "after-angle:%s"%self.vehicle.heading
        #self.getLocation()
    def close(self):

        if self.sitl is not None:
            self.sitl.stop()
        self.vehicle.close()
if __name__=='__main__':
      ran=Ran('/dev/ttyACM0')      #default connect "SITL"
      ran.arm()           #default mode "GUIDED"   armed
      ran.take_off(1)
     
      '''while True:
        ran.goto_by_yaw(heading=20,duration=5)'''
 
            