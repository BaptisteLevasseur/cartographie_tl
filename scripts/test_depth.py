#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge


def callback(img, bridge):
	cv_image =  bridge.imgmsg_to_cv2(img) / 6.0# Pourquoi 6.0 ????
	#cv_image[10,:] = 1.0
	cv2.imshow("depth", cv_image) 
	cv2.waitKey(1)

cv2.namedWindow("depth")
bridge = cv_bridge.CvBridge()

rospy.init_node("subscriber")
rospy.Subscriber("input", Image, lambda m:callback(m,bridge))

rospy.spin()
