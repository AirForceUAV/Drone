#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading,time
# import sys,os
# home=os.path.expanduser('~')
# path=home+"/ObstacleAvoidance"
# sys.path.append(path)

__Organization__='AirForceUAV'
__Author__='mengxz'
__BeginningDate__='2016/5/19'

def _log(message):
	print "[DEBUG]:"+message

def on_connect(client, userdata, rc):
	print("Connected with result code "+str(rc))
	client.subscribe("Command")
	
def on_message(client, userdata, msg):
	eval(str(msg.payload))

def conn_cloud(drone,ip,port=1883):
	import paho.mqtt.client as mqtt	
	client = mqtt.Client(client_id='companion')
	client.on_connect = on_connect
	client.on_message = on_message
	print('Connecting to cloud ... ip={0} port={1}'.format(ip,port))
	client.connect(ip, 1883)
	client.loop_start()
	t1=threading.Thread(target=publisher,args=(client,drone))
	t1.start()
def publisher(client,drone):
	# client.publish('LocationGlobal',drone.LocationGlobal_info())
	# client.publish('Velocity',drone.Velocity_info())
	# client.publish('GPS',drone.GPS_info())
	# client.publish('Battery',drone.Battery_info())
	# client.publish('Heading',drone.Heading_info())
	# client.publish('DistanceFromHome',drone.Distance_from_home())
	# client.publish('DistanceToTarget',drone.Distance_to_target())
	client.publish('CopterStatus',drone.CopterStatus())
		
class Lida(object):
	def __init__(self,drone):
		self.drone=drone

	def go(self):
		target=self.drone.get_target()
		if target is None:
			self.drone._log("Target is None!Please set_target(lat,lon)")
			return 0		
		while True:
			angle_heading=self.drone.Angle_heading_target()
			decision=strategy.Decision(angle_heading)
			distance1=decision[0]
			angle=decision[1]
			distance2=self.drone.get_distance_metres(self.drone.get_location(),target)
			distance=min(distance1,distance2)
			if distance<3:
				self.drone._log("Reached Waypoint!")
				return 1						
			self.drone.fly(distance,angle)
		

	def go_test(self):
		target=self.drone.get_target()
		if target is None:
			self._log("Target is None!Please set_target(lat,lon)")
			return 0	
		angle_heading=self.drone.angle_heading_target()
		decision=strategy.Decision(angle_heading)
		distance1=decision[0]
		angle=decision[1]
		distance2=self.drone.get_distance_metres(self.drone.get_location(),target)
		distance=min(distance1,distance2)
		if distance<3:
			self.drone._log("Reached Waypoint!")
			return 1		
		self.drone.show2()	
		self.drone.fly(distance,angle)
		self.drone.show2()	


if __name__=='__main__':
	#Set up option parsing to get connection string
	import argparse  
	parser = argparse.ArgumentParser(description='Obstacle avoidance system based flight control of open source. ')
	parser.add_argument('--connect', help="vehicle connection target string. 'sitl':sitl;0:/dev/ttyACM0;1:/dev/ttyACM1;If not specified or sitl, SITL is automatically started and used.",default='sitl')
	parser.add_argument('--lida',help="Wether vehicle connect to lida or not;If not specified,is automatically used!",default=1)
	parser.add_argument('--cloud',help='Wether vehicle connect to cloud or not.1/0 is connected/disconnected;If not specified,is automatically Connected!',default=1)
	parser.add_argument('--ip',help="vehicle connection cloud IP.If not specified,139.217.26.207 is automatically used",default='139.217.26.207')	
	parser.add_argument('--port',help="Port Number.Defualt is 1883",default=1883)
	args = parser.parse_args()

	connection_string=args.connect
	cloud=args.cloud
	lida=args.lida
	ip=args.ip
	port=args.port

	from AirForce import Drone
	drone=Drone(connection_string)
	if lida==1:
		from strategy import strategy
		print("Connecting to Lida ...")
		lida=Lida(drone)
	else:
		print('Disconnected to Lida!')
	if cloud==1:
		import paho.mqtt.client as mqtt	
		client = mqtt.Client(client_id='companion',clean_session=True,userdata=None)
		client.reinitialise(client_id='companion',clean_session=True, userdata=None)
		#client.max_inflight_messages_set(1)
		drone.set_client(client)
		client.on_connect = on_connect
		client.on_message = on_message
		print('Connecting to cloud ... ip={0} port={1}'.format(ip,port))

		client.connect(ip, 1883)
		client.loop_start()		
		while True:
			publisher(client,drone)
			time.sleep(1)
	else:
		print('Disconnected to cloud!')

	# drone.arm()
	# drone.takeoff(1)
	# drone.set_channel(3,1300)
	# drone.set_target_metres(0,1000)
	# # print(drone.get_location())
	# while True:		
	# 	raw_input("next")
	# 	if lida.go_test()==1:
	# 		break

	# drone.close()

	# print("Completed")
	
 
			