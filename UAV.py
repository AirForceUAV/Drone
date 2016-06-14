#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,threadpool,math
from AirForce import Drone,CancelWatcher
from pymavlink import mavutil
# import sys,os
# home=os.path.expanduser('~')
# path=home+"/ObstacleAvoidance"
# sys.path.append(path)

__Organization__='AirForceUAV'
__Author__='mengxz'
__BeginningDate__='2016/5/1'
__EndingDate__='2016/6/9'

mqttPool = threadpool.ThreadPool(1)
# eventPool=threadpool.ThreadPool(1)

def _log(message):
	print ">>> "+message

def on_connect(client, userdata, rc):
	_log("Connected mqtt with result code "+str(rc))
	# Subscribe Topic "Command"
	client.subscribe("Command",qos=1)

def eval_wrapper(command):
	# solve namespace problem.
	eval(command)	

def on_message(client, userdata, msg):
	print str(msg.payload)
	if "Cancel" not in str(msg.payload):
		requests = threadpool.makeRequests(eval_wrapper,(str(msg.payload),))
		[mqttPool.putRequest(req) for req in requests]
	else:
		CancelWatcher.Cancel=True
		requests = threadpool.makeRequests(eval_wrapper,("drone.stop()",))
		[mqttPool.putRequest(req) for req in requests]

	# eval(str(msg.payload))


def init_mqtt(ip,port=1883):
	import paho.mqtt.client as mqtt	
	client = mqtt.Client(client_id='companion',clean_session=True,userdata=None)
	client.reinitialise(client_id='companion',clean_session=True, userdata=None)
	client.on_connect = on_connect
	client.on_message = on_message
	client.connect(ip, 1883)
	client.loop_start()		
	return client

def init_sbs():
	from azure.servicebus import ServiceBusService

	# api_key=dict(namespace='AirForceUAV-ns',policy_name='RootManageSharedAccessKey',policy_secret='3bP2rrfIKLbWkQvSwBEJB1iawxhwUdoBC/lDYbRReSI=',host_base='.servicebus.chinacloudapi.cn')
	api_key=dict(namespace='airforceuav',policy_name='RootManageSharedAccessKey',policy_secret='mdq0pk8QTd/VXelOfL7VgQtJQ4Xto2HtVs0rfF2JuOE=',host_base='.servicebus.windows.net')

	sbs = ServiceBusService(api_key["namespace"], shared_access_key_name=api_key["policy_name"], shared_access_key_value=api_key["policy_secret"],host_base=api_key['host_base'])
	return sbs
	
def SendEventStream_wrapper(sbs):
	#solve namespace problem
	SendEventStream(sbs)

def SendEventStream(sbs):
	# print(type(drone),type(sbs))	
	while True:
		message=drone.FlightLog()
		# print(message)
		sbs.send_event('airforceuav',message)
		time.sleep(1)

class Ladar(object):
	def __init__(self,drone):
		self.drone=drone
	
	# Turn yaw ,then forward Flight
	def go(self,defer=1):
		watcher=CancelWatcher()
		target=self.drone.get_target()
		if target is None:
			self.drone._log("Target is None!Please drone.set_target(lat,lon) or drone.set_target_metres(dNorth,dEast).")
			return 0	

		while not watcher.IsCancel():
			distance=round(self.drone.get_distance_metres(self.drone.get_location(),target),2)			
			if distance<3:
				self.drone._log("Reached Target Waypoint!")
				self.drone.stop()
				return 1	
			# angle_heading_target=self.drone.angle_heading_target()
			# decision=strategy.Decision(angle_heading_target)
			# angle=decision[1]
			angle=10
			self.fly(angle)
			time.sleep(defer)
		return 0
	def go_test(self):
		target=self.drone.get_target()
		if target is None:
			self.drone._log("Target is None!Please set_target(lat,lon) or drone.set_target_metres(dNorth,dEast).")
			return 0			
		
		distance=round(self.drone.get_distance_metres(self.drone.get_location(),target),2)			
		if distance<3:
			self.drone._log("Reached Target Waypoint!")
			self.drone.stop()
			return 1	
		angle_heading_target=self.drone.angle_heading_target()
		decision=strategy.Decision(angle_heading_target)
		angle=decision[1]
		# angle=30
		self.fly(angle)	
		return 0


	# forward and right Flight
	def going(self,defer=1):
		watcher=CancelWatcher()
		target=self.drone.get_target()
		if target is None:
			self.drone._log("Target is None!Please drone.set_target(lat,lon) or drone.set_target_metres(dNorth,dEast).")
			return 0	

		while not watcher.IsCancel():
			distance=round(self.drone.get_distance_metres(self.drone.get_location(),target),2)			
			if distance<3:
				self.drone._log("Reached Target Waypoint!")
				self.drone.stop()
				return 1	
			angle_heading_target=self.drone.angle_heading_target()
			decision=strategy.Decision(angle_heading_target)
			angle=decision[1]
			# angle=30
			self.fly_away(angle)
			time.sleep(defer)
		return 0

	def going_test(self):
		target=self.drone.get_target()
		if target is None:
			self.drone._log("Target is None!Please set_target(lat,lon) or drone.set_target_metres(dNorth,dEast).")
			return 0			
		
		distance=round(self.drone.get_distance_metres(self.drone.get_location(),target),2)			
		if distance<3:
			self.drone._log("Reached Target Waypoint!")
			self.drone.stop()
			return 1	
		angle_heading_target=self.drone.angle_heading_target()
		decision=strategy.Decision(angle_heading_target)
		angle=decision[1]
		self.fly_away(angle)	
		return 0
	def fly(self,angle,velocity=1):
		if angle is not 0:
			self.drone.condition_yaw(angle)
		self.send_velocity(velocity,0,0)

	def fly_away(self,angle,velocity=1):
		results=self.parseAngle(angle)
		forward=round(results[1]*velocity,2)
		right=round(results[0]*velocity,2)
		self.send_velocity(forward,right,0)

	def send_velocity(self,forward,right,down):
		vehicle=self.drone.get_vehicle()
		msg=vehicle.message_factory.set_position_target_local_ned_encode(
		0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0,0,0,
		forward,right,down,    #vx,vy,vz
		0,0,0,   #afx,afy,afz
		0,0)    #yaw yaw_rate
		self.drone._log("Forward:{0}m/s;Right:{1}m/s;Down:{2}".format(forward,right,down))
		vehicle.send_mavlink(msg)

	def parseAngle(self,angle):	
		rad=float(angle)/180*math.pi
		return [math.sin(rad),math.cos(rad)]

if __name__=="__main__":
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
	# ladar=Ladar(drone)
	if ladar==1:
		from strategy import strategy
		_log("Connecting to Ladar ...")
		ladar=Ladar(drone)
	else:
		_log('Disconnect to Ladar!')

	if cloud==1:
		# _log('Connecting to Azure by sbs...')
		# sbs=init_sbs()
		# drone.set_sbs(sbs)
		# requests = threadpool.makeRequests(SendEventStream_wrapper,(sbs,))
		# [eventPool.putRequest(req) for req in requests]
		# eventPool.wait()

		_log('Connecting to Azure by mqtt ...')
		mqtt=init_mqtt(ip,port)
		drone.set_mqtt(mqtt)
		while True:
			mqtt.publish('CopterStatus',drone.CopterStatus())
			mqtt.publish('FlightLog',drone.FlightLog())		
			time.sleep(1)
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

	drone.close()

	print("Completed")
	
 
			