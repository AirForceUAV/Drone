#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,threadpool
from AirForce import Drone,CancelWatcher
# import sys,os
# home=os.path.expanduser('~')
# path=home+"/ObstacleAvoidance"
# sys.path.append(path)

__Organization__='AirForceUAV'
__Author__='mengxz'
__BeginningDate__='2016/5/19'

pool = threadpool.ThreadPool(1)

def _log(message):
	print ">>> "+message

def on_connect(client, userdata, rc):
	_log("Connected with result code "+str(rc))
	client.subscribe("Command",qos=1)

def eval_wrapper(command):
	# solve namespace problem.
	eval(command)	

def on_message(client, userdata, msg):
	print str(msg.payload)
	if "Cancel" not in str(msg.payload):
		requests = threadpool.makeRequests(eval_wrapper,(str(msg.payload),))
		[pool.putRequest(req) for req in requests]
	else:
		CancelWatcher.Cancel=True
		requests = threadpool.makeRequests(eval_wrapper,("drone.stop()",))
		[pool.putRequest(req) for req in requests]

	#eval(str(msg.payload))

def conn_cloud(drone,ip,port=1883):
	import paho.mqtt.client as mqtt	
	client = mqtt.Client(client_id='companion',clean_session=True,userdata=None)
	client.reinitialise(client_id='companion',clean_session=True, userdata=None)
	drone.set_mqtt(client)
	client.on_connect = on_connect
	client.on_message = on_message
	_log('Connecting to cloud by mqtt... ip={0} port={1}'.format(ip,port))
	client.connect(ip, 1883)
	client.loop_start()		
	
	from azure.servicebus import ServiceBusService

	api_key=dict(namespace='AirForceUAV-ns',policy_name='RootManageSharedAccessKey',policy_secret='3bP2rrfIKLbWkQvSwBEJB1iawxhwUdoBC/lDYbRReSI=',host_base='.servicebus.chinacloudapi.cn')
	sbs = ServiceBusService(api_key["namespace"], shared_access_key_name=api_key["policy_name"], shared_access_key_value=api_key["policy_secret"],host_base=api_key['host_base'])
	_log('Connecting to eventHub of cloud...')
	# sbs.send_event("airforceuav",drone.Firmware())
	drone.set_eventHub(sbs)
	while True:
		client.publish('CopterStatus',drone.CopterStatus())
		sbs.send_event('airforceuav', drone.FlightLog())
		time.sleep(1)

		
class Ladar(object):
	def __init__(self,drone):
		self.drone=drone

	def go(self):
		watcher=CancelWatcher()
		target=self.drone.get_target()
		if target is None:
			self.drone._log("Target is None!Please drone.set_target(lat,lon) or drone.set_target_metres(dNorth,dEast).")
			return 0	
		while not watcher.IsCancel():
			distance2=round(self.drone.get_distance_metres(self.drone.get_location(),target),2)			
			if distance2<2:
				self.drone._log("Reached Target Waypoint!")
				return 1	
			angle_heading_target=self.drone.angle_heading_target()
			decision=strategy.Decision(angle_heading_target)
			distance1=decision[0]
			angle=decision[1]	
			distance=min(distance1,distance2)				
			self.drone.fly(distance,angle)
		return 0
		

	def go_test(self):
		target=self.drone.get_target()
		if target is None:
			self._log("Target is None!Please set_target(lat,lon)")
			return 0			
		distance2=round(self.drone.get_distance_metres(self.drone.get_location(),target),2)
		
		if distance2<3:
			self.drone._log("Reached Target Waypoint!")
			return 1		
		self.drone.show2()
		angle_heading_target=self.drone.angle_heading_target()
		decision=strategy.Decision(angle_heading_target)
		distance1=decision[0]
		angle=decision[1]	
		distance=min(distance1,distance2)
		self.drone.fly(distance,angle)
		# self.drone.show2()	
		return 0


if __name__=='__main__':
	#Set up option parsing to get connection string
	import argparse  
	parser = argparse.ArgumentParser(description='Obstacle avoidance system based flight control of open source. ')
	parser.add_argument('--connect', help="vehicle connection target string. 'sitl':sitl;0:/dev/ttyACM0;1:/dev/ttyACM1;If not specified or sitl, SITL is automatically started and used.",default='sitl')
	parser.add_argument('--ladar',help="Wether vehicle connect to ladar or not;If not specified,is automatically used!",default=1)
	parser.add_argument('--cloud',help='Wether vehicle connect to cloud or not.1/0 is connected/disconnected;If not specified,is automatically Connected!',default=1)
	parser.add_argument('--ip',help="vehicle connection cloud IP.If not specified,139.217.26.207 is automatically used",default='139.217.26.207')	
	parser.add_argument('--port',help="Port Number.Defualt is 1883",default=1883)
	args = parser.parse_args()

	connection_string=args.connect
	cloud=args.cloud
	ladar=args.ladar
	ip=args.ip
	port=args.port

	
	drone=Drone(connection_string)
	if ladar==1:
		from strategy import strategy
		_log("Connecting to Ladar ...")
		ladar=Ladar(drone)
	else:
		_log('Disconnect to Ladar!')

	if cloud==1:

		conn_cloud(drone,ip,port)
	else:
		_log('Disconnect to Azure!')

	# drone.arm()
	# drone.takeoff(1)
	# drone.set_channel(3,1300)
	# drone.set_target_metres(-100,0)
	# drone.stop()
	# while True:		
	# 	raw_input("next")
	# 	if ladar.go_test()==1:
	# 		break

	# drone.close()

	# print("Completed")
	
 
			