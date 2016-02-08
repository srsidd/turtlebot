#!/usr/bin/env python

import rospy
import numpy as np
import os
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)

		rospy.loginfo("To stop TurtleBot CTRL + C")  
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		rospy.Subscriber('scan', LaserScan, self.laser_callback)
		
		self.r = rospy.Rate(10);
		self.move_cmd = Twist()
		self.move_cmd.linear.x = 0.2
		self.move_cmd.angular.z = 0
		
		while not rospy.is_shutdown():
			self.cmd_vel.publish(self.move_cmd)
			self.r.sleep()
			
	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.move_cmd.linear.x = 0.0
		self.cmd_vel.publish(self.move_cmd)
		rospy.sleep(1)

	def turn(self):
		#self.turning = 1
		#rospy.loginfo("Turn TurtleBot")
		#waittime = 2*np.random.normal(loc=0.0, scale=1.0, size=None)+3
		#t = time.time()
		self.move_cmd.angular.z = .2
		self.move_cmd.linear.x = 0.0
		#while time.time()<t+waittime:
		#	self.cmd_vel.publish(self.move_cmd)
		#	self.r.sleep()
		#self.move_cmd.angular.z = 0
		#self.move_cmd.linear.x = 0.2
		self.cmd_vel.publish(self.move_cmd)
		#self.turning = 0

	def laser_callback(self, scan):
		#def getPosition(self, scan):
		# Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
		self.move_cmd.linear.x = 0.2
		self.move_cmd.angular.z = 0
		depths = []
		max_scan = -1
		too_close = 1
		for dist in scan.ranges:
			if np.isnan(dist):
				depths.append(max_scan)
			else:
				depths.append(dist)
				if dist<too_close:
					#self.move_cmd.linear.x=0
					self.turn()
					break
		#fullDepthsArray = scan.ranges[:]
		#print depths
		#rospy.logdebug('position: {0}'.format(self.position))

if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")

