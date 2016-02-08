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
		self.move_cmd.linear.x = 0.0
		self.move_cmd.angular.z = 0
		
		self.mytime = time.clock()
		self.dt = 0.01
		self.P = 0.0
		self.I = 0.0
		self.D = 0.0
		
		while not rospy.is_shutdown():
			self.cmd_vel.publish(self.move_cmd)
			self.r.sleep()
			
	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.move_cmd.linear.x = 0.0
		self.cmd_vel.publish(self.move_cmd)
		rospy.sleep(1)

	def turn_away(self):
		print "turn away"
		self.move_cmd.angular.z = .3
		self.move_cmd.linear.x = np.max(self.move_cmd.linear.x - 0.08,0)
		
		self.cmd_vel.publish(self.move_cmd)
		#self.turning = 0

	def get_closer(self):

		print "get closer"
		self.move_cmd.angular.z = -.15
		self.move_cmd.linear.x = 0.2
		self.cmd_vel.publish(self.move_cmd)

	def laser_callback(self, scan):
		# Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
		
		depths = []
		max_scan = 100
		for dist in scan.ranges:
			if np.isnan(dist):
				depths.append(max_scan)
			else:
				depths.append(dist)
		dist = np.min(depths)
		
		target = 1
		dx = dist-target
		self.D = self.P-dx/self.dt
		self.I = self.I+dx*self.dt
		self.P = dx	
		#self.move_cmd.linear.x = 0.2
		#self.move_cmd.angular.z = 0
		#if dx<too_close or np.min(depths)==max_scan:
		#	self.turn_away()
		#elif np.min(depths)>too_far:
		#	self.get_closer()
		#else:
		#	print "doing great"
		#	self.cmd_vel.publish(self.move_cmd)

		Kp = .1
		Ki = 0.005
		Kd = 0
		self.move_cmd.angular.z = Ki*self.I #Kp*self.P + Ki*self.I + Kd + self.D
		self.move_cmd.linear.x = 0.2
		self.cmd_vel.publish(self.move_cmd)
		print dist, self.move_cmd.angular.z		
		
		while time.clock()-self.mytime < self.dt:
			pass
			
		self.mytime = time.clock()
		

if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")

