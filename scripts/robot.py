#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData
from cse_190_assi_1.msg import temperatureMessage

from std_msgs.msg import Bool
from std_msgs.msg import String
from read_config import read_config


def callback(data):
   temp = data
   print( temp)

def Robot():
	pub = rospy.Publisher("/temp_sensor/activation", Bool, queue_size = 1)
	rospy.init_node("robot")
	pubbb = True
	pub.publish(pubbb)
	rospy.Subscriber("/temp_sensor/data", temperatureMessage, callback)
	rospy.spin()

if __name__ == '__main__':
    Robot()
