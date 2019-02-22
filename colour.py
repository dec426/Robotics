# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 11:44:42 2019

@author: student
"""

#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        self.image_pub = rospy.Publisher('chatter', String, queue_size = 1)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
        #                                   Image, self.callback)

    def callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((17,29,99)),
                                 numpy.array((81,118,204)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((48,67,134)),
                                 numpy.array((102, 136,228)))

        print numpy.mean(hsv_img[:, :, 0])
        print numpy.mean(hsv_img[:, :, 1])
        print numpy.mean(hsv_img[:, :, 2])

        _, bgr_contours, hierachy = cv2.findContours(
            bgr_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        for c in bgr_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
        print '===='
        
        self.image_pub.publish(str(numpy.mean(cv_image)))        
        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()