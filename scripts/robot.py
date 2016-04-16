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
	self.config = read_config()
	self.text_map = self.config["texture_map"]
	self.move_correct_prob = self.config["prob_move_correct"]
	self.text_correct_prob = self.config["prob_tex_correct"]
	self.prob_array = []
	self.current_pos = self.config["starting_pos"]
	self.init_prob()
	#create publisher to active temp_sensor/activation
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
		self.handle_temp
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

	'''self.terminator = rospy.Publisher(
		'/map_node/sim_complete',
		Bool,
		queue_size = 1
	)'''

	#reqeust texture from requesttexture
	self.texture_request = rospy.ServiceProxy(
		"requestTexture",
		requestTexture
	)
	self.temp_data = np.float32(0.0)
	self.temp_texture = "s"
	rospy.spin()
	#self.sensor_loop()	
	#rospy.sleep(1)

    def handle_temp(self, data):
	#returning temperature data
	self.temp_data = data.temperature
	#debug massage
	print self.temp_data
	self.write_temp.publish(self.temp_data)
	self.handle_texture()
	self.handle_move()

    '''def sensor_loop(self):
	while not rospy.is_shutdown():
	    #make texture publish
	    self.handle_texture()
	    #make move 
	    self.handle_move() '''
		
    def handle_move(self):
	self.nextMove = self.move()
	print self.nextMove
	#self.handle_move_prob()

    def handle_move_prob(self):
  	return

    def handle_texture(self):
	self.texture_data = self.texture_request()
	#store texture data
	self.write_texture.publish(self.texture_data.data)
	#debug message
	print self.texture_data.data
	self.handle_texture_probability()

    def handle_texture_probability(self):
	total = 0
        for i in range (self.rows):
		for j in range (self.columns):
			texture = self.text_map[i][j]
			if texture == self.texture_data.data:
				position = self.columns*i + j
				self.prob_array[position] *= self.text_correct_prob
				total += self.prob_array[position]
				#print self.prob_array[position]
			else:
				position = self.columns*i + j
				self.prob_array[position] *= (1-self.text_correct_prob)
				total += self.prob_array[position]
				#print self.prob_array[position]

	for x in range (self.rows):
		for y in range (self.columns):
			position = self.columns*x + y
			self.prob_array[position] = self.prob_array[position]/total	
			#print self.prob_array[position]


    def init_prob(self):
	self.rows = len(self.config['pipe_map'])
	self.columns = len(self.config['pipe_map'][0])
	self.size = np.float32(self.rows*self.columns)
        for i in range (self.size):
		self.prob_array.append(1/(self.size))	
		#print self.prob_array[i]

def handle_write_toFile(self):
	terminator = rospy.Publisher(
		'/map_node/sim_complete',
		Bool,
		queue_size = 1
	)
	terminator.publish(self)

if __name__ == '__main__':
   r = Robot()
   handle_write_toFile(True)
