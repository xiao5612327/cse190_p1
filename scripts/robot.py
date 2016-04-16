#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData
from cse_190_assi_1.msg import * 
from cse_190_assi_1.srv import *
from std_msgs.msg import Bool
from std_msgs.msg import String
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
	
	rospy.spin()

    def handle_call_texture(self, data):
	#reqeust texture from requesttexture
	self.texture_request = rospy.ServiceProxy("requestTexture", requestTexture)
	#returning temperature data
	temp_data = data.temperature
	print temp_data
	return temp_data


if __name__ == '__main__':
   r = Robot()
