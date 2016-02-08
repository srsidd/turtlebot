#!/usr/bin/env python

import rospy
import numpy as np
import os
import sys
from cv2 import * 

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kobuki_msgs.msg import BumperEvent
from imreg import *
import copy

import time

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)

		rospy.loginfo("To stop TurtleBot CTRL + C")  
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		
		###----------- IMAGE ----------###
		# turtlebot
		#self.image_sub = rospy.Subscriber("/camera/depth/image",Image, self.callback, queue_size = 1)
		# gazebo
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.callback, queue_size = 1)
		#self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image, self.callback, queue_size = 1)
		self.bridge = CvBridge()
		self.time = time.time()
		

		rospy.Subscriber('scan', LaserScan, self.laser_callback, queue_size = 1)
		rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback, queue_size = 1)
		#rospy.Subscriber('scan', LaserScan, self.laser_callback)

		#print scan
		scan = np.random.uniform(low=-10.0, high=10.0, size=(640,1))
		self.scanim = self.scan2im(scan)
		self.counter = 0

		self.r = rospy.Rate(10);
		
		self.move_cmd = Twist()
		#self.move_cmd.linear.x = 0.2
		#self.move_cmd.angular.z = 0

		self.bridge = CvBridge()
		
		while not rospy.is_shutdown():
			#self.cmd_vel.publish(self.move_cmd)
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

	def reverse(self):
		self.move_cmd.angular.z = .0
		self.move_cmd.linear.x = -0.2
		self.cmd_vel.publish(self.move_cmd)
		rospy.sleep(1.5)


	def laser_callback(self, scan):
		#def getPosition(self, scan):
		# Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
		#self.move_cmd.linear.x = 0.2
		#self.move_cmd.angular.z = 0
		depths = []
		max_scan = -1
		too_close = 1
		for dist in scan.ranges:
			if np.isnan(dist):
				depths.append(max_scan)
			else:
				depths.append(dist)
				'''
				if dist<too_close:
					#self.move_cmd.linear.x=0
					self.turn()
					break
				'''
		
		# save scan 
		scanname = 'scan'+'.m'
		logfile = open(scanname, 'w')               #make a file, a+ will append 
		logfile.write("scan=[")
		for m in range(len(depths)):
			logfile.write(str(depths[m])+" ")
		logfile.write("];\n\n")    
		logfile.close()  

		# convert scan to image
		self.scan = self.scan2im(depths)

		# rotation and scale invariance		
		im0 = imread('model_fixed.png',0)
		target_hist = np.load('hist.npy')
		self.scan = self.scan.astype(np.uint8)
		im2, scale, angle, (t0, t1) = similarity(im0, self.scan)
		im2 = im2.astype(np.uint8)

		# calculate score 
		i0 = im0.astype(np.float32)/np.max(im0.reshape(np.size(im0),1))#255.0
		i2 = im2.astype(np.float32)/255.0
		# ensure the alignment is actually an alignment
		merge = (i0*i2)*255.0 #element wise multiplication
		merge = merge.astype(np.uint8)

		# extract scene
		scene = (i2-i0)*255.0 
		scene[scene<0] = 0.0;
		scene = scene.astype(np.uint8)		

		# find center of object
		i0_bin = i0>0.5
		cy,cx = np.nonzero(i0_bin)
		mcx = np.mean(cx)
		mcy = np.mean(cy)
		i0center = copy.deepcopy(i0)
		#print cy,cy+5, cx,cx+5, i0center.shape
		i0center[mcy:mcy+5,mcx:mcx+5]=1.0
		'''
		i0255 = i0*255
		gray = i0255.astype(np.uint8)
		rgb = cvtColor(gray,COLOR_GRAY2BGR)
		circle(rgb, (cx,cy), 5, cv.Scalar(255, 255, 0)) 
		'''

		# find distance center of object
		scene_bin = scene>0.5
		cy,cx = np.nonzero(scene_bin)
		dists = np.zeros((cy.size))
		for i in range(cy.size):
			dists[i] = np.sqrt((cy[i]-mcy)**2 + (cx[i]-mcx)**2)
		#print dists

		# create histogram
		
		hist, bin_edges = np.histogram(dists, bins=100, range=(0,100))

		h0 = np.zeros((hist.size,2))
		h0[:,0] = np.array(hist)
		h0[:,1] = np.array(bin_edges[:-1])
		#np.save('hist', np.array(h0))

		h1 = cv.fromarray(h0.astype(np.float32))
		h2 = cv.fromarray(target_hist.astype(np.float32))
	
		emd = cv.CalcEMD2(h2, h1, cv.CV_DIST_L2) +abs(np.sum(h0[:,0])-np.sum(target_hist[:,0]))
		#print emd

		#visualize
		imcat = np.hstack((i0*255.0,self.scan))
		imcat = np.hstack((imcat,i2*255.0))
		#imcat = np.hstack((imcat,scene))
		#imcat = np.hstack((imcat,i0center*255))

		score = merge.reshape(merge.size,1)
		sumscore = 0.0
		maxval = 0.0
		for i in range(np.size(score)):
			if score[i]>maxval:
				maxval = score[i]
			if score[i]>0:
				sumscore += (score[i]+0.0)/255.0

		# Write some Text
		font = FONT_HERSHEY_SIMPLEX
		#if score
		origin = (180,20)
		fontscale = .5
		thickness = 1
		
		# print imcat.dtype
		gray = imcat.astype(np.uint8)
		rgb = cvtColor(gray,COLOR_GRAY2BGR)
		
		mystring = str(sumscore)
		if sumscore>20:
			putText(rgb,mystring,origin, font, fontscale,((0,0,255)),thickness)
			# update model
			updaterate = .99
			newim = (updaterate*i0+(1-updaterate)*i2)*255.0
			newim = newim.astype(np.uint8)
			imwrite('model.png', newim)
			#imshow("scan", newim)
		else:
			putText(rgb,mystring,origin, font, fontscale,((255,0,0)),thickness)

		origin = (180,80)
		mystring = str(emd)
		if emd<40:
			putText(rgb,mystring,origin, font, fontscale,((0,255,0)),thickness)
		else:
			putText(rgb,mystring,origin, font, fontscale,((255,0,0)),thickness)

		imshow("scan", rgb)
		waitKey(1)
		

	def callback(self, image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image)
		except CvBridgeError, e:
			print e

		#print 'fps: ', 1.0/(time.time()-self.time)
		#self.time = time.time()

		imshow("Test", cv_image)
		#cv2.imshow("Test", rgb_im)

		waitKey(1)

	def bumper_callback(self, bumper):
		#def getPosition(self, scan):
		# Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
		#self.move_cmd.linear.x = 0.2
		#self.move_cmd.angular.z = 0
		#bumper.bumper
		#bumper.state

		
		if bumper.state==1:
			#self.move_cmd.linear.x=0
			print "BUMP"
			#self.reverse()
			#break

	def scan2im(self, s):
		# get angle
		nums =range(640)
		nums = np.asarray(nums)+0.0
		nums[:] = (nums - 320.0)/640.0#[(x - 320.0)/640.0 for x in nums]
		fov = .34
		nums[:] = nums*(fov*np.pi)#[x*(.34*np.pi) for x in nums]
		nums = nums.reshape(nums.size,1)

		# remove nans
		#print "s",s
		tmp1 = s>0
		valids = []
		validnum = []
		#print np.size(s) 640
		for i in range(np.size(s)):
			if s[i]>0:
				valids.append(s[i])
				validnum.append(nums[i])
		valids = np.array(valids)
		valids = valids.reshape(np.size(valids),1)
		validnum = np.array(validnum)
		validnum  = validnum .reshape(np.size(validnum ),1)


		# convert to cartesian
		#print np.shape(validnum), np.shape(valids) (639, 1) (639, 1)
		ex, why = pol2cart(validnum,valids)
		# make an image
		imsize = 100
		im = np.zeros((imsize,imsize))

		ex = ex-np.min(ex) 
		why = why-np.min(why) 

		tmp = np.array([np.max(ex), np.max(why)])
		maxval = np.max(tmp)
		ex = ex*((imsize-1.0)/maxval)
		why = why*((imsize-1.0)/maxval)
		
		#print ex.shape (639, 1)
		for i in range(np.size(ex)):
			# make sure value falls in valid range 0-99
			'''
			if i>= np.size(ex) or i>=np.size(why):
				print i,
				print": ",why,ex,"  MERP!"
			else:
				print i,
				print": ",why,ex,"  ",": ",why[i,0],ex[i,0],"  ",
			'''
			tmp = np.array([why[i,0],0])
			y = int(np.max(tmp))
			tmp = np.array([ex[i,0],0])
			x = int(np.max(tmp))
			
			im[imsize-1-y,x] = 255#1;

		return im

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(phi, r):
    x = r * np.cos(phi)
    y = r * np.sin(phi)
    return(x, y)

if __name__ == '__main__':
	np.set_printoptions(precision=3)
	#np.set_printoptions(suppress=True)
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")

