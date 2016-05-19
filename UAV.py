#!/usr/bin/env python
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt

__college__='PKU'
__Author__='mengxz'
__BeginningDate__='2016/5/19'



def on_connect(client, userdata, rc):
	print("Connected with result code "+str(rc))
	client.subscribe("Command")

def on_message(client, userdata, msg):
	eval(str(msg.payload))

if __name__=='__main__':
	#Set up option parsing to get connection string
	import argparse  
	parser = argparse.ArgumentParser(description='Obstacle avoidance system based flight control of open source. ')
	parser.add_argument('--connect', help="vehicle connection target string. If not specified or sitl, SITL is automatically started and used.",default='sitl')
	parser.add_argument('--ip',help="vehicle connection cloud IP.If not specified,139.217.26.207 is automatically used",default='139.217.26.207')
	args = parser.parse_args()

	connection_string=args.connect
	ip=args.ip

	from AirForce import Drone
	drone=Drone(connection_string)
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message
	print('Connecting to cloud ... ip={0} port={1}'.format(ip,1883))
	client.connect(ip, 1883)
	client.loop_forever()
	
 
			