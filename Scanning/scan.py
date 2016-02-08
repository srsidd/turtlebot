
import rospy
import roslib
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

import time

class Test:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image",Image, self.callback, queue_size = 1)
        #self.image_pub = rospy.Publisher("test_image", Image, queue_size = 1)
        self.bridge = CvBridge()
        self.time = time.time()


        while(1==1):
			im0 = imread('im1.png',0) #numpy ndarray -1 = alpha, 0 = gray, 1 = color
			im1 = imread("im2.png",0)

			im2, scale, angle, (t0, t1) = similarity(im0, im1)
			#print scale, angle
			#imshow("Test", im2)

			scan = np.random.uniform(low=-10.0, high=10.0, size=(640,1))
			#print scan
			im = self.scan2im(scan)

			imshow("Test", im)
			waitKey(1)


    def callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError, e:
            print e

        #np_arr = np.fromstring(image.data, np.uint8) #only 1 channel?
        np_arr = np.fromstring(cv_image, np.uint8)
        #print cv_image.shape
        print 'fps: ',1.0/(time.time()-self.time)
        self.time = time.time()
        
        n,m = cv_image.shape
        k = 3
        im = np.reshape(cv_image,(np.size(cv_image),1))
        y,x = np.meshgrid(range(n),range(m))
        triples = (np.reshape(cv_image,(np.size(x),1)), np.reshape(cv_image,(np.size(x),1)), im)
 

        imshow("Test", cv_image)
        #cv2.imshow("Test", rgb_im)
    
        waitKey(1)
        
    def scan2im(self, s):
		# get angle

		nums =range(640)
		nums = np.asarray(nums)+0.0
		nums[:] = (nums - 320.0)/640.0#[(x - 320.0)/640.0 for x in nums]
		nums[:] = nums*(.34*np.pi)#[x*(.34*np.pi) for x in nums]
		nums = nums.reshape(nums.size,1)

		# remove nans
		tmp1 = s>0

		## where is broken
		#ind = np.where(tmp1>0)
		#print 's',s,"inds", ind[0] #makes a second vector for some reason
		#print "s",type(s), np.size(s)
		#print "ind",type(ind), np.size(ind)
		#ind = ind[0]
		#valid = s[ind] 
		valids = []
		validnum = []
		print np.size(s)
		for i in range(np.size(s)):
			if i>0:
				valids.append(s[i])
				validnum.append(nums[i])
		valids = np.array(valids)
		validnum = np.array(validnum)

		# convert to cartesian
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
		
		#print ex.shape
		for i in range(np.size(ex)):
			tmp = np.array([why[i,0],0])
			y = int(np.max(tmp))
			tmp = np.array([ex[i,0],0])
			x = int(np.max(tmp))
			im[y,x] = 1;

		return im

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)


def main(args):
    test = Test()
    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)