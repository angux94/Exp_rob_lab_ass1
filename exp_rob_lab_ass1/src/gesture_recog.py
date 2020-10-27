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

cb_msg = None

def coord_select():
	#world plotting, for refence on the waypoints selection
	plt.title('Select play destination')
	plt.figure(1)
	for i in range(0,10):
		plt.plot([i*100, i*100], [0,1000], 'k')
		plt.plot([0,1000],[i*100, i*100], 'k')

	#waypoints input
	xy = plt.ginput(1)
	plt.close()
	plt.show()

	return xy


def callback(data):
	global cb_msg
	cb_msg = data.data
	

def main():
	global cb_msg
	rospy.init_node('Gesture_recognition')
	pub = rospy.Publisher('Play_coords', Point, queue_size=10)
	sub = rospy.Subscriber('Command', String, callback)

	play_coord = Point(x = 0, y = 0)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		
		if(cb_msg == "play"):
			cb_msg = None
			xy = coord_select()
			#storage of the x and y coords of the point
			x = xy[0][0]
			y = xy[0][1]

			play_coord = Point(x = x, y = y)
			pub.publish(play_coord)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	main()