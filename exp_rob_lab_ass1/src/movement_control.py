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




# Global variables
cb_point = None

# Callback functoins
def callback(data):
	global cb_point
	cb_point = data


# main
def main():
	"""Main code for movement control

	Receives some geometry_msgs.Point coordinates and takes a time to arrive to the destination.
	Sends back a signal to inform the arrival.

	subscribe: /move_coords (geometry_msgs.Point)
	publish: /arrived (std_msgs.Bool)
	"""
	rospy.init_node('Movement_control')

	# Publishers and Subscribers
	pub = rospy.Publisher('arrived', Bool, queue_size=10)
	sub = rospy.Subscriber('move_coords', Point, callback)

	rate = rospy.Rate(10) # 10hz

	# Initialiizations
	global cb_point
	robot_coords = Point(x = 10, y = 10)

	while not rospy.is_shutdown():

		# Mke sure flag is down
		arrive = False

		#If we receive a coordinate
		if(cb_point):

			# Wait some travel time
			time.sleep(5)
			robot_coords.x = cb_point.x
			robot_coords.y = cb_point.y
			print(cb_point)
			print("Robot arrived")
			
			# Set flag
			arrive = True

			# Clean coordinates
			cb_point = None

			pub.publish(arrive)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	main()