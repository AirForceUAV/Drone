#!/usr/bin/env python
# -*- coding: utf-8 -*-

__college__='PKU'
__Author__='mengxz'
__BeginningDate__='2016/5/19'



def on_connect(client, userdata, rc):
	print("Connected with result code "+str(rc))
	client.subscribe("Command")

def on_message(client, userdata, msg):
	eval(str(msg.payload))

class Lida(object):
	def __init__(self,drone):
		self.drone=drone

	def go(self):
		target=self.drone.get_target()
		if target is None:
			self.drone._log("Target is None!Please set_target(lat,lon)")
			return 0		
		while True:
			angle_heading=self.drone.get_angle_heading()
			decision=strategy.Decision(angle_heading)
			distance1=decision[0]
			angle=decision[1]
			distance2=self.drone.get_distance_metres(self.drone.get_location(),target)
			distance=math.min(distance1,distance2)
			if distance<0.2:
				self.drone._log("Reached Waypoint!")
				return 1						
			self.drone.fly(distance,angle)
		

	def go_test(self):
		target=self.drone.get_target()
		if target is None:
			self._log("Target is None!Please set_target(lat,lon)")
			return 0	
		angle_heading=self.drone.get_angle_heading()
		print("Angle_heading_target:{0}".format(angle_heading))	
		decision=strategy.Decision(angle_heading)
		distance1=decision[0]
		angle=decision[1]
		distance2=self.drone.get_distance_metres(self.drone.get_location(),target)
		distance=math.min(distance1,distance2)
		if distance<0.2:
			self.drone._log("Reached Waypoint!")
			return 1			
		self.drone.fly(distance,angle)


if __name__=='__main__':
	#Set up option parsing to get connection string
	import argparse  
	parser = argparse.ArgumentParser(description='Obstacle avoidance system based flight control of open source. ')
	parser.add_argument('--connect', help="vehicle connection target string. 'sitl':sitl;0:/dev/ttyACM0;1:/dev/ttyACM1;If not specified or sitl, SITL is automatically started and used.",default='sitl')
	parser.add_argument('--ip',help="vehicle connection cloud IP.If not specified,139.217.26.207 is automatically used",default='139.217.26.207')
	parser.add_argument('--cloud',help='Wether vehicle connect to cloud or not.1/0 is connected/disconnected;If not specified,is automatically Connected!',default=1)
	parser.add_argument('--lida',help="Wether vehicle connect to lida or not;If not specified,is automatically used!",default=1)
	args = parser.parse_args()

	connection_string=args.connect
	cloud=args.cloud
	lida=args.lida

	from AirForce import Drone
	drone=Drone(connection_string)
	if cloud==1:
		import paho.mqtt.client as mqtt
		ip=args.ip
		client = mqtt.Client()
		client.on_connect = on_connect
		client.on_message = on_message
		print('Connecting to cloud ... ip={0} port={1}'.format(ip,1883))
		client.connect(ip, 1883)
		client.loop_forever()
	else:
		print('disconnected to cloud!')

	if lida==1:
		from strategy import strategy
		print("Connecting to Lida ...")
		lida=Lida(drone)
	else:
		print('disconnected to Lidad!')
	# sitl:-35.363261,149.165230
	# drone.set_target(-35.38,149.184)
	# drone.set_target(10,10)
	# lida.go()
	

	
 
			