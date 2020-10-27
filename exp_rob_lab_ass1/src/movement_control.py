#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

cb_point = None

def callback(data):
	global cb_point
	cb_point = data

def main():

	rospy.init_node('Movement_control')
	pub = rospy.Publisher('Arrived', Bool, queue_size=10)
	sub = rospy.Subscriber('Play_coords', Point, callback)

	rate = rospy.Rate(10) # 10hz

	global cb_point
	robot_coords = Point(x = 10, y = 10)

	while not rospy.is_shutdown():
		
		arrive = False
		if(cb_point):
			time.sleep(5)
			robot_coords.x = cb_point.x
			robot_coords.y = cb_point.y
			print(cb_point)
			print("Robot arrived")
			arrive = True
			cb_point = None

			pub.publish(arrive)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	main()