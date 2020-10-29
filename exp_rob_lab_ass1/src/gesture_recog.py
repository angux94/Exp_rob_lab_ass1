#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt




# Global variables
cb_msg = None


def coord_select():
	"""Function to display a grid and select the destination point

	Returns
	---------
	xy
		X and Y coordinates of the point selected
	"""

	# 1000x1000 grid for selecting the destination
	plt.title('Select play destination')
	plt.figure(1)
	for i in range(0,10):
		plt.plot([i*100, i*100], [0,1000], 'k')
		plt.plot([0,1000],[i*100, i*100], 'k')

	# Point input
	xy = plt.ginput(1)
	plt.close()
	plt.show()

	return xy

# Callback functions
def callback(data):
	global cb_msg
	cb_msg = data.data
	

def main():
	
	"""Main code for gesture recognition

	After receiving the "play" command, displays a 1000x1000 grid as the workspace to select a play destination.
	Publishes the selected coordinates

	subscribe: /gesture_request (std_msgs.String)
	publish: /move_coords (geometry_msgs.Point)
	"""
	rospy.init_node('Gesture_recognition')

	# Publishers and Subscribers
	pub = rospy.Publisher('move_coords', Point, queue_size=10)
	sub = rospy.Subscriber('gesture_request', String, callback)

	# Initialiizations
	global cb_msg
	play_coord = Point(x = 0, y = 0)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		
		# Check if we have a command
		if(cb_msg == "play"):
			# Clean variable
			cb_msg = None

			# Get destination coordinates
			xy = coord_select()

			#storage of the x and y coords of the point
			x = xy[0][0]
			y = xy[0][1]

			# Publish coordinates
			play_coord = Point(x = x, y = y)
			pub.publish(play_coord)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	main()