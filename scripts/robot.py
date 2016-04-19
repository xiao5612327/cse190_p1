#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTexture, moveService
from cse_190_assi_1.msg import temperatureMessage,RobotProbabilities 
from std_msgs.msg import Bool
from std_msgs.msg import String, Float32
from read_config import read_config



class Robot():
    def __init__(self):
	rospy.init_node("robot")
	self.config = read_config()
	self.text_map = self.config["texture_map"]
	self.pipe_map = self.config["pipe_map"]
	self.move_correct_prob = self.config["prob_move_correct"]
	self.text_correct_prob = self.config["prob_tex_correct"]
	self.sd = self.config["temp_noise_std_dev"]
	self.move_list = self.config["move_list"]
	self.prob_array = []
	self.output_prob_array = []
	self.current_pos = self.config["starting_pos"]
	self.init_prob()

	#create publisher to active temp_sensor/activation
	self.active_pub = rospy.Publisher(
		"/temp_sensor/activation", 
		Bool, 
		queue_size = 1
	)
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
	#write probabilities into file
	self.write_prob = rospy.Publisher(
		'/results/probabilities',
		RobotProbabilities,
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
	#final publish call to create output file
	self.terminator = rospy.Publisher(
		'/map_node/sim_complete',
		Bool,
		queue_size = 1
	)
	rospy.sleep(1)
	self.active_pub.publish(True)

	self.prob = RobotProbabilities()
	self.temp_data = np.float32(0.0)
	self.move_made = 0
	self.total_move = len(self.move_list)
	self.active_pub.publish(True)
	rospy.spin()
	#self.sensor_loop()	
	#rospy.sleep(1)

    def handle_temp(self, data):
	#returning temperature data
	self.temp_data = data.temperature
	print self.temp_data
	self.updateTempBelief()
	#debug massage
	self.handle_texture()
	self.handle_move_prob()
	self.make_move()
	self.write_all()

    '''def sensor_loop(self):
	while not rospy.is_shutdown():
	    #make texture publish
	    self.handle_texture()
	    #make move 
	    self.handle_move() '''

    def updateTempBelief(self):
	total = 0
        for i in range (self.rows):
		for j in range (self.columns):
			truth_value = self.pipe_map[i][j]
			if( truth_value == self.pipe_map[0][2] ):
				truth_value = 40
			elif( truth_value == self.pipe_map[0][1] ):
				truth_value = 25
			elif( truth_value == self.pipe_map[0][0] ):
				truth_value = 20
			position = self.columns*i + j
			self.prob_array[position] *= self.temp_prob_given_pos(truth_value)
			total += self.prob_array[position]

	for x in range (self.rows):
		for y in range (self.columns):
			position = self.columns*x + y
			self.prob_array[position] = self.prob_array[position]/total	

    def temp_prob_given_pos(self, truth_value):
	value1 = 2 * math.pi
	value2 = math.sqrt(value1) * self.sd
	value3 = np.float32(1/value2)
	value4 = ((self.temp_data - truth_value) * (self.temp_data - truth_value)) * -1
	value5 = np.float32(value4/(2 * self.sd * self.sd))	
	value6 = value3 * math.pow(math.e, value5)	
	return value6

    def write_all(self):
	#publish self.temp_data
	self.write_temp.publish(self.temp_data)
	#publish texture data
	self.write_texture.publish(self.texture_data.data)
	print self.texture_data.data
	self.write_prob.publish(self.prob_array)
	#publish prob data
	if self.move_made > self.total_move:
	    self.handle_write_toFile()
	    rospy.sleep(1)
	    rospy.signal_shutdown(Robot)
    
    def make_move(self):
	self.move_made = self.move_made + 1
	self.move(self.try_to_move)

    def handle_move_prob(self):
	#debug
	#print size
	if self.move_made >= self.total_move:
	    return
	current_move = self.move_list[self.move_made]
	self.try_to_move = current_move

	#loop thought all prob array	
        for i in range (0, self.rows):
	    for j in range (0, self.columns):
	        self.current_pos[0] = i
		self.current_pos[1] = j
	    	self.calculate_correct_move(current_move)
	    	#publish prob data
	for k in range (self.size):
	    self.prob_array[k] = self.output_prob_array[k] 
	    self.output_prob_array[k]  = 0.0
    
    def calculate_correct_move(self, move):
	#get x and y position
	current_x = self.current_pos[0] + move[0]
	if current_x == self.rows: 
	    current_x = 0
	elif current_x < 0:
	    current_x = self.rows - 1
	current_y = self.current_pos[1] + move[1]
	if current_y == self.columns:
	    current_y = 0
	elif current_y < 0:
	    current_y = self.columns - 1
	#calculate prob about move correct and incorrect
	move_position = self.columns * current_x + current_y
	current_position = self.columns * self.current_pos[0] + self.current_pos[1]
	
	#destination grid prob = current grid prob * correct prob + dest_guid prob
	dest_prb = self.prob_array[current_position] * self.move_correct_prob
	self.output_prob_array[move_position] = self.output_prob_array[move_position] + dest_prb

 	#get all possible move and remove the one correct	
	possible_move = deepcopy(self.config['possible_moves'])
	possible_move.remove(move)
	size = len(possible_move)
	#loop 4 times to calculate other 4 incorrect move
	for i in range (size):
	    next_incor_move = possible_move[i]
	    self.calculate_incorrect_move(next_incor_move)	

    def calculate_incorrect_move(self, move):
	#get x and y position
	current_x = self.current_pos[0] + move[0]
	if current_x == self.rows: 
	    current_x = 0
	elif current_x < 0:
	    current_x = self.rows - 1
	current_y = self.current_pos[1] + move[1]
	if current_y == self.columns:
	    current_y = 0
	elif current_y < 0:
	    current_y = self.columns - 1
	#calculate prob about move correct and incorrect
	move_position = self.columns * current_x + current_y
	current_position = self.columns * self.current_pos[0] + self.current_pos[1]
	#destination grid prob = current grid prob * correct prob + dest_guid prob
	dest_prb = self.prob_array[current_position] * (1 - self.move_correct_prob)
	self.output_prob_array[move_position] = self.output_prob_array[move_position] + dest_prb	
	
    def handle_texture(self):
	self.texture_data = self.texture_request()
	#debug message
	#print self.texture_data.data
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
		self.output_prob_array.append(0.0)	
		#print self.prob_array[i]

    def handle_write_toFile(self):
	self.terminator.publish(True)

if __name__ == '__main__':
   r = Robot()

