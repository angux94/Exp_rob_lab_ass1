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


def main():
	rospy.init_node('Command_recognition')
	pub = rospy.Publisher('Command', String, queue_size=10)
	
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():

		txt = raw_input("Write PLAY to commmand: ")
		if(txt == "Play" or txt == "play" or txt == "PLAY"):
			print("Valid command")
			txt = "play"
			pub.publish(txt)
		else:
			print("Your command '" + txt + "' is not valid.")
			print("Please write a valid command")
			print("")
			continue
		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
	main()