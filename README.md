##AirForce.py Class Drone --API

#Get
```bash
get_vehicle() 
get_alt()            --current altitude
get_home()       --home_location
get_location()    --current_location , global_relative_frame Object
get_heading() 
get_bearing(self,aLocation1, aLocation2)
get_angle(self,lat,lon):
```

#Log and Flight Status
```bash
FC_info(self)
UAV_info(self)
```

#Set
```bash
set_target_metres(self,dNorth,dEast):
set_target(lat,lon)
set_airspeed(self,airspeed)
set_groundspeed(self,groundspeed)
set_roi(location)
get_parameter(self,key):
set_parameter(self,key,value)
```

#Formal
```bash
arm(self)
disarm(self)
takeoff(self,alt=5)
```

#Guided and movement
```bash
condition_yaw(self,heading, relative=True)
goto_NED(self,dNorth,dEast)
goto(self,location)    --location=[lat,lon]
fly(self,distance,heading=0,velocity=1.0)
fly_position(self,distance,heading=0):
go_home(self)
```

#Basic control
```bash
forward(self,distance=0.5,velocity=1.0)
backward(self,distance=0.5,velocity=1.0)
left(self,distance=0.5,velocity=1.0)
right(self,distance=0.5,velocity=1.0)
up(self,distance=0.5,velocity=1.0)
down(self,distance=0.5,velocity=1.0)
stop(self)    --Brake
yaw_left(self,angle=3)
yaw_right(self,angle=3)
```

#Flight Mode
```bash
RTL(self)
Land(self)
Stab(self)
Loiter(self)
Althold(self)
get_mode(self)
```

#Landing Gear
```bash
deploy(self)    --Deploy Landing Gear
retract(self)     --Retract Landing Gear
```

#Channel
```bash
get_channels(self)
get_channel(self,key)
set_channel(self,key,value)
```


#Other
```bash
show()   --Distance to home:{0},heading:{1},lat:{2},lon:{3},alt:{4}
show2()
close()
```

##UAV.py -- API

#Option
```bash
--connect= 'sitl'(default) or  0(/dev/ttyACM0)  or  1(/dev/ttyACM1)
--cloud=1(default) or 0
--ip='139.217.26.207'
--lida=1(default) or 0
```
#Function
```bash
lida.go()
lida.got_test()
```
