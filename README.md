AirForce.py --API

--Get
#get_vehicle(self) 
#get_alt(self)            --current altitude
#get_home(self)       --home_location
#get_location(self)    --current_location , global_relative_frame Object
#get_heading(self) 
#get_bearing(self,aLocation1, aLocation2)
#get_angle(self,lat,lon):

--Log and Flight Status
#FC_info(self)
#UAV_info(self)

--Set
set_target_metres(self,dNorth,dEast):
set_target(lat,lon)
#set_airspeed(self,airspeed)
#set_groundspeed(self,groundspeed)
#set_roi(location)
#get_parameter(self,key):
#set_parameter(self,key,value)

--arm ,disarm ,take_off
arm(self)
disarm(self)
takeoff(self,alt=5)

--Guided and movement
#condition_yaw(self,heading, relative=True)
#goto_NED(self,dNorth,dEast)
#goto(self,location)    --location=[lat,lon]
fly(self,distance,heading=0,velocity=1.0)
#fly_position(self,distance,heading=0):
go_home(self)

--8 directions control
forward(self,distance=0.5,velocity=1.0)
backward(self,distance=0.5,velocity=1.0)
left(self,distance=0.5,velocity=1.0)
right(self,distance=0.5,velocity=1.0)
up(self,distance=0.5,velocity=1.0)
down(self,distance=0.5,velocity=1.0)
stop(self)    --Brake
yaw_left(self,angle=3):
yaw_right(self,angle=3):

--Flight Mode
RTL(self)
Land(self)
Stab(self)
Loiter(self)
Althold(self)
get_mode(self)

--Landing Gear
deploy(self)    --Deploy Landing Gear
retract(self)     --Retract Landing Gear

--Channel
get_channels(self)
#get_channel(self,key):
#set_channel(self,key,value):
#default_channel(self,key):
#default_channels(self):

--Other
show(self):   --Distance to home:{0},heading:{1},lat:{2},lon:{3},alt:{4}
close(self)


UAV.py -- API

#Maing Portal
#--connect= 'sitl'(default) or  0(/dev/ttyACM0)  or  1(/dev/ttyACM1)
#--cloud=1(default) or 0
#--ip='139.217.26.207'
#--lida=1(default) or 0
lida.go()
lida.got_test()