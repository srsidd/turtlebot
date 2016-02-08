#!/usr/bin/env python
import sys, time
import numpy as np
#from scipy.ndimage import filters
import roslib
import rospy

#import cv2              # cv2.foo
from cv2 import *       # foo

import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import time

class Test:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image",Image, self.callback, queue_size = 1)
        rospy.Subscriber('scan', LaserScan, self.laser_callback, queue_size = 1)
        #self.image_pub = rospy.Publisher("test_image", Image, queue_size = 1)
        self.bridge = CvBridge()
        self.time = time.time()
        self.prevscan = []

    def callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError, e:
            print e

        print 'fps: ', 1.0/(time.time()-self.time)
        self.time = time.time()

        imshow("Test", cv_image)
        #cv2.imshow("Test", rgb_im)
    
        waitKey(1)
        

        #test2 = cv2.resize(cv_image,(250,240))
        #print test2.shape 
        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(test2))
    
    def laser_callback(self, scan):
        #def getPosition(self, scan):
        # Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
        depths = []
        max_scan = 0
        too_close = 1
        for dist in scan.ranges:
            if np.isnan(dist):
                depths.append(max_scan)
            else:
                depths.append(dist)
                '''
                if dist<too_close:
                    #self.move_cmd.linear.x=0
                    print "too close"
                    break
                '''
        sig1 = cv.CreateMat(len(depths), 2, cv.CV_32FC1)

        for h in range(len(depths)):
            cv.Set2D(sig1, h, 0, cv.Scalar(depths[h]))
            cv.Set2D(sig1, h, 1, cv.Scalar(h))

        #This is the important line were the OpenCV EM algorithm is called
        try:
            merp = cv.CalcEMD2(sig1, self.prevscan, cv.CV_DIST_L2)
            print merp
        except:
            self.prevscan = sig1
            print 'hopefully this only prints once'
        

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