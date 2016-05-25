#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading,time
import sys,os
home=os.path.expanduser('~')
path_Avoidance=home+"/ObstacleAvoidance"
sys.path.append(path)

__Organization__='AirForceUAV'
__Author__='mengxz'
__BeginningDate__='2016/5/19'



def on_connect(client, userdata, rc):
	print("Connected with result code "+str(rc))
	client.subscribe("Command")

def on_message(client, userdata, msg):
	eval(str(msg.payload))

def conn_cloud(ip,port=1883):
	import paho.mqtt.client as mqtt
	
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message
	print('Connecting to cloud ... ip={0} port={1}'.format(ip,port))
	client.connect(ip, 1883)
	client.loop_forever()
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
			if distance<1:
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
		if distance<1:
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
	parser.add_argument('--ip',help="vehicle connection cloud IP.If not specified,139.217.26.207 is automatically used",default='139.217.26.207')
	parser.add_argument('--cloud',help='Wether vehicle connect to cloud or not.1/0 is connected/disconnected;If not specified,is automatically Connected!',default=1)
	parser.add_argument('--lida',help="Wether vehicle connect to lida or not;If not specified,is automatically used!",default=1)
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
		t1=threading.Thread(target=conn_cloud,args=(ip,port))
		t1.start()	
		t1.join()
	else:
		print('Disconnected to cloud!')

	# drone.arm()
	# drone.takeoff(5)
	# drone.set_target_metres(-10,0)
	# print(drone.get_location())
	# lida.go()
	# drone.stop()
	# drone.condition_yaw(0,False)
	# drone.fly(5,180,1)

	# time.sleep(10)
	# drone.close()

	# print("Completed")
	
 
			