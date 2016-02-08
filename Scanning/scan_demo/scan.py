
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
			im0 = imread('rect1.png',0) #numpy ndarray -1 = alpha, 0 = gray, 1 = color
			im1 = imread("rect2.png",0)
			
			im2, scale, angle, (t0, t1) = similarity(im0, im1)
			#print scale, angle
			imshow("Test", im2)
			
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
 

        cv2.imshow("Test", cv_image)
        #cv2.imshow("Test", rgb_im)
    
        cv2.waitKey(1)
        


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