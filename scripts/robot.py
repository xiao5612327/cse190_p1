#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.srv import *
from std_msgs.msg import Bool
from std_msgs.msg import String
from read_config import read_config


def call_Texture(data):
   	temp = data
   	print ( temp)

	response = rospy.ServiceProxy("requestTexture", requestTexture)
	texture_reading = response

def Robot():
	pub = rospy.Publisher("/temp_sensor/activation", Bool, queue_size = 1)
	rospy.init_node("robot")
	rospy.sleep(1)
	pubbb = True
	pub.publish(pubbb)
	

	rospy.Subscriber("/temp_sensor/data", temperatureMessage, call_Texture)
	
	//just added you can modify. it is trying to make move
	move_step = rospy.ServiceProxy("moveService", moveService)
	make_move(move_step[0], move_step[1])

	

	rospy.spin()
def make_move(x, y):
	return	

if __name__ == '__main__':
    Robot()
