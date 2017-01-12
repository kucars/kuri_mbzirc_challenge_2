#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 10/04/2016
@author: Abdullah Abduldayem (template)
@author: Husameldin Mukhtar 
"""

import sys
import rospy
import cv2
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, RegionOfInterest
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from kuri_mbzirc_challenge_2_msgs.msg import WrenchDetectionAction
from scipy.spatial import distance
from collections import OrderedDict
import numpy as np
from numpy.linalg import inv
from matplotlib import pyplot as plt
from pylab import *
from imutils import perspective
from imutils import contours
import imutils
from datetime import datetime
import time
import os
import glob
from os.path import expanduser
home = expanduser("~")

node_name = "wrench_detection_server"
topic_name = "wrench_detection"

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)
 
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)
 
	# return the edged image
	return edged

class WrenchDetectionServer:

    def __init__(self):
        self.is_node_enabled = True

        # Set up callbacks
	self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback, queue_size=1)

        # Start actionlib server
        #self.server = actionlib.SimpleActionServer(topic_name, WrenchDetectionAction, self.execute, False)
        #self.server.start()

        rospy.loginfo("Started wrench detection node. Currently on standby")


    def execute(self, goal_msg):
        # This node was called, perform any necessary changes here
        self.is_node_enabled = True
        rospy.loginfo("Wrench detection node enabled")


    def camera_callback(self, data):
        # Return if node is not enabled
        if (not self.is_node_enabled):
            return


        # Node is enabled, process the camera data
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	img = cv_image.copy()
	output = img.copy()

	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	#thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)[1]
	thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,3,2)

	# perform edge detection, then perform a dilation + erosion to close gaps in between object edges
	edged = cv2.Canny(blurred, 20, 150)
	edged = cv2.dilate(edged, None, iterations=1)
	edged = cv2.erode(edged, None, iterations=1)
	edged = cv2.erode(edged, None, iterations=1)

	edged2 = auto_canny(blurred)
	edged3 = cv2.dilate(edged2.copy(), None, iterations=1)
	edged4 = cv2.erode(edged3.copy(), None, iterations=1)

	cv2.imshow("thresh", thresh)
	cv2.waitKey(10)

	cv2.imshow("edged4", edged4)
	cv2.waitKey(10)

	# find contours in the thresholded image and initialize the shape detector
	#cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = cv2.findContours(edged4.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]

	# sort the contours from left-to-right and initialize the
	(cnts, _) = contours.sort_contours(cnts)

	#zlab = ToolLabeler()

	# loop over the contours
	for c in cnts:
		# compute the center of the contour, then detect the name of the
		# shape using only the contour
		M = cv2.moments(c)
		#cX = int((M["m10"] / M["m00"]))
		#cY = int((M["m01"] / M["m00"]))
	
	 
		# multiply the contour (x, y)-coordinates by the resize ratio,
		# then draw the contours and the name of the shape on the image
		c = c.astype("float")
		c *= 1
		c = c.astype("int")
		text = "{}".format(shape)

		# if the contour is not sufficiently large, ignore it
		if cv2.contourArea(c) < 80:
			continue

		cv2.drawContours(output, [c], -1, (0, 255, 0), 2)
	 
		# compute the rotated bounding box of the contour
		#orig = image.copy()
		box = cv2.minAreaRect(c)
		box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
		box = np.array(box, dtype="int")
	 
		# order the points in the contour such that they appear
		# in top-left, top-right, bottom-right, and bottom-left
		# order, then draw the outline of the rotated bounding
		# box
		box = perspective.order_points(box)
		cv2.drawContours(output, [box.astype("int")], -1, (0, 255, 0), 2)

		(tl, tr, br, bl) = box

		dA = distance.euclidean(tl, bl)
		dB = distance.euclidean(tl, tr)

		#size = zlab.label(np.array([dA,dB]))

		#print(int(bl[0]))

		cv2.putText(output, "({:d},{:d})".format(int(dA),int(dB)), (int(bl[0])-15,int(bl[1])+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 2)
		#cv2.putText(output, size, (int(bl[0])-15,int(bl[1])+55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
	 
		# loop over the original points and draw them
		for (x, y) in box:
			cv2.circle(output, (int(x), int(y)), 5, (0, 0, 255), -1)

	
	 
		# show the output image
		cv2.imshow("Image", output)
		cv2.waitKey(10)
	


        # If the wrenches are detected, return the region of interest
        #p1 = Point(0 ,0 ,0)
        #p2 = Point(10,0 ,0)
        #p3 = Point(10,10,0)
        #p4 = Point(0 ,10,0)

        #rospy.loginfo("Found wrench")
        #result = WrenchDetectionResult()

        #result.ROI = [p1, p2, p3, p4]
        #self.server.set_succeeded(result)

        # Disable the node since it found its target
        #self.is_node_enabled = False



if __name__ == '__main__':
      rospy.init_node(node_name)
      server = WrenchDetectionServer()
      rospy.spin()
