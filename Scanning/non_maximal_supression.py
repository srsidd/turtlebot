#!/usr/bin/env python

import rospy
import numpy as np
import os
import sys
from cv2 import * 

import matplotlib.pyplot as plt
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

		self.prob_map = imread('prob_map.png',0)
		maxes = self.non_max(self.prob_map)
		self.r = rospy.Rate(10);

		while not rospy.is_shutdown():
			
			cy,cx = np.nonzero(maxes)
			#index = np.argmax(self.combined)
			#y,x = np.unravel_index(index, np.shape(self.combined))
			
			num_pts = np.size(cx)
			dists = np.zeros((num_pts,num_pts))

			#compute distances
			for i in range(num_pts):
				for j in range(num_pts):
					dists[i,j] = ((cy[i]-cy[j])**2+(cx[i]-cx[j])**2)**0.5
					dists[j,i] = dists[i,j]
			
			xpts,ypts,bestpath = solveTSP(cx, cy, dists)
			plotpath(xpts,ypts,bestpath)
			
			#imshow("Test", maxes)
			#waitKey(1)


			self.r.sleep()

	def non_max(self,im):
		imT = im.T

		M,N = np.shape(im)
		#print M,N #480 512
		maxes = np.zeros((M,N))

		w = 15
		for m in range(w,M-w):
			for n in range(w,N-w):
				window = im[(m-w):(m+w), (n-w):(n+w)]
				#print np.max(window.reshape((np.size(window),1)))
				if np.max(window.reshape((np.size(window),1)))==im[m,n] and np.max(window.reshape((np.size(window),1)))>0:
					maxes[m,n]=255
				'''
				#print m,n
				if im[m,n]>im[m,n-1] and im[m,n]>im[m,n+1] and imT[n,m]>imT[n,m-1] and imT[n,m]>imT[n,m+1]:
					#print m,n,im[m,n],im[m,n-1],im[m,n+1],imT[n,m],imT[n,m-1],imT[n,m+1]
					maxes[m,n]=255
				'''
		#maxes = nonMaximalSupress1(im,(20,20))
		#print np.shape(im)
		return maxes


	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		rospy.sleep(1)

def solveTSP(xpts, ypts, dist):

	cities = np.size(xpts)
	bestpath = np.zeros((cities,1))
	bestpathcost = np.zeros((cities,1))
	visited = np.zeros((cities,1))
	current_node = 0
	bestscore = 0

	# Nearest Neighbor
	c = 0.0001*np.random.normal(loc=0.0, scale=1.0, size = (dist.shape))
	dist = dist+c #break ties
	for i in range(cities-1):
		bestpath[i] = current_node
		visited[current_node,0] = 1
		inds = visited<1
		tmp = dist[:, current_node]
		tmp = tmp.reshape(np.size(tmp),1)
		val = np.min(tmp[inds]) #min of not yet visited
		current_node = np.where(tmp==val)[0][0]
		bestpathcost[i,0] = val
		bestscore += val

	bestpath[cities-1] = current_node
	bestpath = bestpath.astype(np.uint8)
	bestpathcost[cities-1] = dist[bestpath[cities-1,0],bestpath[0,0]]
	#print bestpathcost[cities-1]
	#plotpath(xpts,ypts,bestpath)

	# Local Search
	counter = 0
	swapbenefit = 1
	while swapbenefit>0:
		#print swapbenefit
		counter = counter+1
		#edgepairs = np.zeros((cities,cities))
		neworder = bestpath
		newcost = bestpathcost
		swapbenefit = 0
		for i in range(cities):
			for j in range(i+1,cities):
				#NOTE: edgepairs(j,i) = dist(bestpath(i),bestpath(i+1)) + dist(bestpath(j),bestpath(J)) 
				edgepairs = bestpathcost[i,0] + bestpathcost[j,0]
				J = j+1
				if j+1==cities:
					J = 0
				swapcost = dist[bestpath[i,0],bestpath[j,0]] + dist[bestpath[i+1,0],bestpath[J,0]]
				#print edgepairs[j,i], swapcost
				if edgepairs>swapcost:
					benefit = edgepairs - swapcost

					if benefit>swapbenefit: # greedy swap
						#print 'swap'
						if j+1>cities:
							tmp = np.concatenate((range(i+1),range(j,i,-1)))
							tmp = tmp.reshape(np.size(tmp),1) 
							tmp = tmp.astype(np.uint8)
							neworder = bestpath[tmp,0] 
							tmp = np.concatenate((range(i),range(j,i-1,-1)))
							tmp = tmp.reshape(np.size(tmp),1)
							tmp = tmp.astype(np.uint8)
							newcost = bestpathcost[tmp,0]
						else:
							#tmp = np.concatenate(range(i),range(j,i+1,-1), range(J,cities)])
							tmp = np.concatenate((range(i+1),range(j,i,-1)))
							tmp = np.concatenate((tmp,range(J,cities)))
							tmp = tmp.reshape(np.size(tmp),1)
							tmp = tmp.astype(np.uint8)
							neworder = bestpath[tmp,0] 
							tmp = np.concatenate((range(i),range(j,i-1,-1)))
							tmp = np.concatenate((tmp, range(J,cities)))
							tmp = tmp.reshape(np.size(tmp),1)
							tmp = tmp.astype(np.uint8)
							newcost = bestpathcost[tmp,0]

						newcost[i] = dist[bestpath[i],bestpath[j]]
						newcost[j] = dist[bestpath[i+1],bestpath[J]]
						swapbenefit = benefit

		bestpath = neworder
		bestpathcost = newcost
		#plotpath(xpts,ypts,bestpath)

	return xpts,ypts,bestpath
	#print bestpath, bestpathcost

def plotpath(xpts,ypts,path):
	font1 = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 10,
        }
	font2 = {'family': 'serif',
        'color':  'blue',
        'weight': 'normal',
        'size': 10,
        }


	plt.cla()   
	plt.clf()  
	#plt.close()
	cities = np.size(xpts)
	oldCoordinate = np.array([xpts[path[0]], ypts[path[0]]])

	for j in range(1,cities):
		newCoordinate = [xpts[path[j]], ypts[path[j]]]

		plt.plot(newCoordinate[0], newCoordinate[1], 'ok')
		plt.text(newCoordinate[0]+15, newCoordinate[1]+10, str(j), fontdict=font1)		
		plt.text(newCoordinate[0], newCoordinate[1]+10, str(path[j][0]), fontdict=font2)
		myline = np.array([oldCoordinate, newCoordinate])
		plt.plot(myline[:,0], myline[:,1],'m')
		oldCoordinate = newCoordinate

	#complete the cycle
	newCoordinate = [xpts[path[0]], ypts[path[0]]]
	plt.plot(newCoordinate[0], newCoordinate[1], 'ok')
	plt.text(newCoordinate[0]+15, newCoordinate[1]+10, str(0), fontdict=font1)		
	plt.text(newCoordinate[0], newCoordinate[1]+10, str(path[0][0]), fontdict=font2)
	myline = np.array([oldCoordinate, newCoordinate])
	plt.plot(myline[:,0], myline[:,1],'m')
	plt.show()

if __name__ == '__main__':
	np.set_printoptions(precision=3)
	#np.set_printoptions(suppress=True)
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")

