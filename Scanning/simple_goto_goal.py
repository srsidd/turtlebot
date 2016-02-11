#!/usr/bin/env python

import rospy
import numpy as np
import os
import sys
from cv2 import * 

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kobuki_msgs.msg import BumperEvent
from actionlib_msgs.msg import GoalStatusArray
from imreg import *
import copy

import time

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)

		rospy.loginfo("To stop TurtleBot CTRL + C")  
		rospy.on_shutdown(self.shutdown)
		self.set_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
		self.Posesub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amclCallback)

		rospy.Subscriber('/move_base/status', GoalStatusArray, self.check_status, queue_size = 1)
		
		self.r = rospy.Rate(10)
		self.goal1 = PoseStamped()
		self.goal1.header.stamp = rospy.Time.now()
		self.goal1.header.frame_id = "map"
		self.goal1.pose.position.x = -2.8
		self.goal1.pose.position.y = 2.0
		self.goal1.pose.position.z = 0.0
		self.goal1.pose.orientation.x = 0.0
		self.goal1.pose.orientation.y = 0.0
		self.goal1.pose.orientation.z = 0.0
		self.goal1.pose.orientation.w = 1.0

		self.goal2 = PoseStamped()
		self.goal2.header.stamp = rospy.Time.now()
		self.goal2.header.frame_id = "map"
		self.goal2.pose.position.x = -2.5
		self.goal2.pose.position.y = 6.0
		self.goal2.pose.position.z = 0.0
		self.goal2.pose.orientation.x = 0.0
		self.goal2.pose.orientation.y = 0.0
		self.goal2.pose.orientation.z = -0.773942270111
		self.goal2.pose.orientation.w = 0.633256158704
		
		self.currentgoal = self.goal1

		self.myPosition = PoseWithCovarianceStamped()

		rospy.sleep(3)
		
		while not rospy.is_shutdown():
			self.r.sleep()
			
	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		rospy.sleep(1)

	def check_status(self, robostatus):
		setgoal = 0
		nogoal = len(robostatus.status_list)==0

		if nogoal:
			setgoal = 1
		elif robostatus.status_list[0].text=='Goal reached.':
			setgoal = 1

		if setgoal:
			self.set_goal.publish(self.currentgoal)
			
			# Sometimes the status lies
			if self.samePose(self.myPosition, self.currentgoal):		
				if self.currentgoal == self.goal1:
					self.currentgoal = self.goal2
				else:
					self.currentgoal = self.goal1


	def amclCallback(self, data):
		self.myPosition.pose.pose.position.x = data.pose.pose.position.x
		self.myPosition.pose.pose.position.y = data.pose.pose.position.y
		self.myPosition.pose.pose.position.z = data.pose.pose.position.z
		self.myPosition.pose.pose.orientation.x = data.pose.pose.orientation.x
		self.myPosition.pose.pose.orientation.y = data.pose.pose.orientation.y
		self.myPosition.pose.pose.orientation.z = data.pose.pose.orientation.z
		self.myPosition.pose.pose.orientation.w = data.pose.pose.orientation.w
			
	def samePose(self,poscov,posstamp):
		d1 = poscov.pose.pose.position.x - posstamp.pose.position.x
		d2 = poscov.pose.pose.position.y - posstamp.pose.position.y
		d3 = poscov.pose.pose.position.z - posstamp.pose.position.z
		'''
		d4 = poscov.pose.pose.orientation.x - posstamp.pose.orientation.x
		d5 = poscov.pose.pose.orientation.y - posstamp.pose.orientation.y
		d6 = poscov.pose.pose.orientation.z - posstamp.pose.orientation.z
		d7 = poscov.pose.pose.orientation.w - posstamp.pose.orientation.w
		'''
		d = np.sqrt(d1**2+d2**2+d3**2)#+d4**2+d5**2+d6**2+d7**2)
		if d<0.5:
			return 1
		else:
			0 

if __name__ == '__main__':
	np.set_printoptions(precision=3)
	#np.set_printoptions(suppress=True)
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")

'''

rostopic pub /move_base_simple/goal geometry_msgsoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: -2.8, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'

'''
