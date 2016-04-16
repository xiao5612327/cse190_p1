#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTexture, moveService
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
from std_msgs.msg import Bool
from std_msgs.msg import String, Float32
from read_config import read_config



class Robot():
    def __init__(self):
	rospy.init_node("robot")
	#createa publisher to active temp_sensor/activation
	self.active_pub = rospy.Publisher(
		"/temp_sensor/activation", 
		Bool, 
		queue_size = 1
	)
	rospy.sleep(1)
	#publish
	self.active_pub.publish(True)
	#create a subscriber to recive data from temp_sensor/data
	self.temp_sensor_sub = rospy.Subscriber(
		"/temp_sensor/data", 
		temperatureMessage, 
		self.handle_call_texture
	)
	#request map server to make move
	self.move = rospy.ServiceProxy(
		"moveService", 
		moveService
	)
	#write temperture data into file
	self.write_temp = rospy.Publisher(
		'/results/temperature_data',
		Float32,
		queue_size = 10
	)
	#write texture into file
	self.write_texture = rospy.Publisher(
		'/results/texture_data',
		String,
		queue_size = 10
	)
	#reqeust texture from requesttexture
	self.texture_request = rospy.ServiceProxy(
		"requestTexture",
		requestTexture
	)
	self.temp_data = np.float32(0.0)
	self.temp_texture = "s"
	self.sensor_loop()	
	rospy.sleep(1)

    def handle_call_texture(self, data):
	#returning temperature data
	self.temp_data = data.temperature
	#debug massage
	print self.temp_data
	self.write_temp.publish(self.temp_data)

    def sensor_loop(self):
	while not rospy.is_shutdown():
	    #make texture publish
	    self.handle_texture()
	    #make move 
	    self.handle_move()
		
    def handle_move(self):
	temp = self.move()
	print temp 

    def handle_texture(self):
	texture_data = self.texture_request()
	#store texture data
	self.write_texture.publish(texture_data.data)
	#debug message
	print texture_data.data

if __name__ == '__main__':
   r = Robot()
