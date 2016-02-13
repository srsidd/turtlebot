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

		# Data Setup
		self.prob_map = imread('prob_map.png',0)
		self.visited = 255*np.ones(np.shape(self.prob_map))
		im_map = imread('mymap.pgm',0)
		self.map_mask = im_map>240
		self.visited = self.visited*self.map_mask
		self.combined = self.visited.astype(np.float32)+self.prob_map.astype(np.float32)
		self.cam_im = copy.deepcopy(self.prob_map)
		self.disp_im = copy.deepcopy(self.prob_map)

		self.move_cmd = Twist()
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		
		###----------- IMAGE ----------###
		self.bridge = CvBridge()
		# turtlebot
		#self.image_sub = rospy.Subscriber("/camera/depth/image",Image, self.callback, queue_size = 1)
		# gazebo
		rospy.Subscriber("/camera/rgb/image_raw",Image, self.callback, queue_size = 1)
		#self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image, self.callback, queue_size = 1)
		
		scan = np.random.uniform(low=-10.0, high=10.0, size=(640,1))
		self.scanim = self.scan2im(scan)
		rospy.Subscriber('scan', LaserScan, self.laser_callback, queue_size = 1)
		#rospy.Subscriber('scan', LaserScan, self.laser_callback)

		rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback, queue_size = 1)
		
		self.myPosition = PoseWithCovarianceStamped()
		self.currentgoal = PoseStamped()
		self.currentgoal.header.stamp = rospy.Time.now()
		self.currentgoal.header.frame_id = "map"
		self.currentgoal.pose.position.x = 0.0
		self.currentgoal.pose.position.y = 0.0
		self.currentgoal.pose.position.z = 0.0
		self.currentgoal.pose.orientation.x = 0.0
		self.currentgoal.pose.orientation.y = 0.0
		self.currentgoal.pose.orientation.z = 0.0
		self.currentgoal.pose.orientation.w = 1.0
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amclCallback)
		
		rospy.Subscriber('/move_base/status', GoalStatusArray, self.check_status, queue_size = 1)
		
		self.set_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
		self.firststuck = 1

		self.object_score = 0.0
		self.scene_score = 0.0

		self.goalnum = 0
		self.imnum = 0

		self.time = time.time()
		self.r = rospy.Rate(10);

		while not rospy.is_shutdown():
			#self.cmd_vel.publish(self.move_cmd)
			self.r.sleep()
			
	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		rospy.sleep(1)


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
		
		# save scan 
		scanname = 'scan'+'.m'
		logfile = open(scanname, 'w')               #make a file, a+ will append 
		logfile.write("scan=[")
		for m in range(len(depths)):
			logfile.write(str(depths[m])+" ")
		logfile.write("];\n\n")    
		logfile.close()  

		# convert scan to image
		self.scanim = self.scan2im(depths)

		# rotation and scale invariance		
		im0 = imread('model_fixed.png',0)
		self.scanim = self.scanim.astype(np.uint8)
		im2, scale, angle, (t0, t1) = similarity(im0, self.scanim)
		im2 = im2.astype(np.uint8)

		self.object_score, self.scene_score = self.scan_score(im0, im2)



		'''
		#visualize
		imcat = np.hstack((i0*255.0,self.scanim))
		imcat = np.hstack((imcat,i2*255.0))

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
		'''

	def callback(self, image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image)
		except CvBridgeError, e:
			print e

		#print 'fps: ', 1.0/(time.time()-self.time)
		#self.time = time.time()
		self.cam_im = cv_image
		#imshow("Test", cv_image)
		#cv2.imshow("Test", rgb_im)

		#waitKey(1)

	def bumper_callback(self, bumper):
		#bumper.bumper
		#bumper.state
		
		if bumper.state==1:
			#self.move_cmd.linear.x=0
			print "BUMP"
			#self.reverse()
			#break

	def check_status(self, robostatus):
		setgoal = 0
		stuck = 0
		nogoal = len(robostatus.status_list)==0

		if nogoal:
			setgoal = 1
		elif robostatus.status_list[0].text=='Goal reached.':
			setgoal = 1
		elif robostatus.status_list[0].text=='Failed to find a valid plan. Even after executing recovery behaviors.':
			print 'this is as far as I go\n'#, self.myPosition.pose.pose.position, '\n->\n', self.currentgoal.pose.position
			setgoal = 1
			stuck = 1
			# remove unattainable point
			self.deadPoint(self.currentgoal.pose.position.x, self.currentgoal.pose.position.y)
		else:
			self.firststuck = 1

		#print robostatus.status_list[0].text
		if setgoal:
			self.set_goal.publish(self.currentgoal)
			
			# Sometimes the status lies
			if self.samePose(self.myPosition, self.currentgoal) or (stuck and self.firststuck):	
				# find new goal	
				self.combined = self.visited.astype(np.float32)*self.map_mask.astype(np.float32)+self.prob_map.astype(np.float32)*self.map_mask.astype(np.float32)
				index = np.argmax(self.combined)
				y,x = np.unravel_index(index, np.shape(self.combined))
			
				# transform prob im pt to map
				mappt = self.prob2map(x,y)

				self.currentgoal.header.stamp = rospy.Time.now()
				self.currentgoal.pose.position.x = mappt[0]
				self.currentgoal.pose.position.y = mappt[1]
				
				self.firststuck = 0 #race condition

				self.goalnum += 1
				self.saveData('_GOAL')
				print 'new goal:', mappt


	def amclCallback(self, data):
		self.myPosition.pose.pose.position.x = data.pose.pose.position.x
		self.myPosition.pose.pose.position.y = data.pose.pose.position.y
		self.myPosition.pose.pose.position.z = data.pose.pose.position.z
		self.myPosition.pose.pose.orientation.x = data.pose.pose.orientation.x
		self.myPosition.pose.pose.orientation.y = data.pose.pose.orientation.y
		self.myPosition.pose.pose.orientation.z = data.pose.pose.orientation.z
		self.myPosition.pose.pose.orientation.w = data.pose.pose.orientation.w

		# update map of visited locations
		self.visitPoint(data.pose.pose.position.x, data.pose.pose.position.y)
		
		self.combined = self.visited.astype(np.float32)*self.map_mask.astype(np.float32) + self.prob_map.astype(np.float32)*self.map_mask.astype(np.float32)
		self.combined = self.combined.astype(np.float32)/np.max(self.combined.reshape(np.size(self.combined),1))
		self.combined = self.combined*255

		#show current goal
		mappt = self.map2prob(self.currentgoal.pose.position.x,self.currentgoal.pose.position.y)
		#self.combined[mappt[1]-5:mappt[1]+5,mappt[0]-5:mappt[0]+5]=255
		rgb = cvtColor(self.combined.astype(np.uint8),COLOR_GRAY2BGR)
		circle(rgb, (int(mappt[0]),int(mappt[1])), 10, cv.Scalar(0, 0, 255)) 

		imcat = np.hstack((self.visited.astype(np.uint8),self.combined.astype(np.uint8)))

		#SHOW SCORES
		font = FONT_HERSHEY_SIMPLEX
		origin = (180,20)
		fontscale = .5
		thickness = 1
		mystring = str(self.object_score)
		if self.object_score>20:
			putText(rgb,mystring,origin, font, fontscale,((0,0,255)),thickness)
		else:
			putText(rgb,mystring,origin, font, fontscale,((255,0,0)),thickness)

		origin = (180,80)
		mystring = str(self.scene_score)
		if self.scene_score<40:
			putText(rgb,mystring,origin, font, fontscale,((0,255,0)),thickness)
		else:
			putText(rgb,mystring,origin, font, fontscale,((255,0,0)),thickness)

		if self.object_score>20:
			self.saveData('_OBJ')
		elif self.scene_score<40:
			self.saveData('_SCENE')

		self.disp_im = rgb

		imshow("Test", rgb)
		waitKey(1)


	def visitPoint(self,x,y):
		ytop, xtop = np.shape(self.visited)
		probpt = self.map2prob(x,y)

		visit_rad = 20
		xmin = np.maximum(0,np.floor(probpt[0])-visit_rad)
		xmax = np.minimum(xtop,np.floor(probpt[0])+visit_rad)
		ymin = np.maximum(0,np.floor(probpt[1])-visit_rad)
		ymax = np.minimum(ytop,np.floor(probpt[1])+visit_rad)

		self.visited[ymin:ymax,xmin:xmax] = 0
		print 'VISIT: map',x,y,'   prob',probpt[0],probpt[1],    'goal:', self.currentgoal.pose.position.x,self.currentgoal.pose.position.y

	def deadPoint(self,x,y):
		ytop, xtop = np.shape(self.visited)
		probpt = self.map2prob(x,y)

		visit_rad = 20
		xmin = np.maximum(0,np.floor(probpt[0])-visit_rad)
		xmax = np.minimum(xtop,np.floor(probpt[0])+visit_rad)
		ymin = np.maximum(0,np.floor(probpt[1])-visit_rad)
		ymax = np.minimum(ytop,np.floor(probpt[1])+visit_rad)

		self.map_mask[ymin:ymax,xmin:xmax] = 0;
		print 'killed:'#,xmin,xmax,ymin,ymax

	def map2prob(self,x,y):
		mappt = np.array([x, y, 1])
		trans2prob = np.array([[20.2232, 0.0065], [0.1045, -20.2959], [244.0298, 235.9429]])
		probpt = np.dot(mappt.T,trans2prob)
		probpt = probpt.reshape(2,)
		return probpt

	def prob2map(self,x,y):
		probpt = np.array([x, y, 1])
		probpt = probpt.reshape(3,1)
		trans2map = np.array([[0.0494, 0.0000], [0.0003, -0.0493],[-12.1264, 11.6210]])
		mappt = np.dot(probpt.T,trans2map)
		mappt = mappt.reshape(2,)
		return mappt

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

	def scan_score(self, im_template, im_alignedscan):
		
		""" OBJECT SCORE """
		i0 = im_template.astype(np.float32)/np.max(im_template.reshape(np.size(im_template),1))#255.0
		i2 = im_alignedscan.astype(np.float32)/255.0
		# ensure the alignment is actually an alignment
		merge = (i0*i2)*255.0 #element wise multiplication
		merge = merge.astype(np.uint8)

		score = merge.reshape(merge.size,1)
		sumscore = 0.0
		maxval = 0.0
		for i in range(np.size(score)):
			if score[i]>maxval:
				maxval = score[i]
			if score[i]>0:
				sumscore += (score[i]+0.0)/255.0


		""" ENVIRONMENT SCORE """
		target_hist = np.load('hist.npy')

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

		return sumscore, emd

	def saveData(self, reason):
		mystring = 'images/im_'
		deets =  str(self.imnum) + '__togoal_'+ str(self.goalnum) + '__ob_'+ str(self.object_score) + '__scene_'+ str(self.scene_score) + reason +'.png'
		imwrite('images/im_'+deets, self.cam_im)
		imwrite('images/disp_'+deets, self.disp_im)
		imwrite('images/scan_'+deets, self.scanim)
		self.imnum += 1

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

