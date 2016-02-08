#!/usr/bin/env python


import rospy



'''
Listen and Talk
by David Isele

'''


import roslib
import rospy
import os
import time
from kobuki_msgs.msg import SensorState
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class my_process():

	def __init__(self):
		# setup
		rospy.init_node("my_process")	
		pub = rospy.Publisher('my_topic', String, queue_size=10)
		rospy.Subscriber("/image_converter/output_video",SensorState,self.SensorPowerEventCallback)
		rate = rospy.Rate(10) # 10hz

		rospy.spin();


	def SensorPowerEventCallback(self,data):
		print data

		my_message = "hey!"

		rospy.loginfo(my_message)
		pub.publish(my_message)
		rate.sleep()

if __name__ == '__main__':
	try:
		my_process()
	except rospy.ROSInterruptException:
		rospy.loginfo("exception")
