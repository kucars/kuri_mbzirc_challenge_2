#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 10/04/2016
@author: Husameldin Mukhtar
@author: Abdullah Abduldayem (template) 
"""

import sys
import rospy
import rospkg
import cv2
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseStamped
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

clos_kernel_sz = 5;

act_wrs_w = float(0.29)

#===========================================================================================
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('kuri_mbzirc_challenge_2_wrench_detection')

img_s = cv2.imread(pkg_path + '/images/wrench_3.PNG')
out_s = img_s.copy()

#cv2.imshow("img_s", img_s)
#cv2.waitKey(10)

gray_s = cv2.cvtColor(img_s, cv2.COLOR_BGR2GRAY)
thresh_s = cv2.threshold(gray_s, 128, 255, cv2.THRESH_BINARY_INV)[1]

#cv2.imshow("thresh_s", thresh_s)
#cv2.waitKey(10)

#cnts = cv2.findContours(thresh1.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts_s = cv2.findContours(thresh_s.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
cnts_s = cnts_s[0] if imutils.is_cv2() else cnts_s[1]
#=============================================================================================

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
		self.camera_sub = rospy.Subscriber('/softkinetic_camera/rgb/image_mono', Image, self.camera_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)
		#self.camera_sub = rospy.Subscriber('/softkinetic_camera/rgb/image_color', Image, self.camera_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

		self.image_width = 1
		self.image_height = 1
		self.cameraInfo_sub = rospy.Subscriber("/softkinetic_camera/rgb/camera_info",CameraInfo,self.get_camera_info, queue_size = 1)

		self.tool_ROI = RegionOfInterest()
		self.tool_ROI_pub = rospy.Publisher("/ch2/detection/tool/bb_pixel", RegionOfInterest, queue_size = 1)

		self.tool_pos = PoseStamped()
		self.tool_pos_pub = rospy.Publisher("/ch2/detection/tool/center_pose", PoseStamped, queue_size = 1)

		# Start actionlib server
		#self.server = actionlib.SimpleActionServer(topic_name, WrenchDetectionAction, self.execute, False)
		#self.server.start()

		rospy.loginfo("Started wrench detection node. Currently on standby")

		self.tool_size = '14mm'

		
		sizes = OrderedDict({
				"13mm": (2.8, 16.8),
				"14mm": (3, 17.9),
				"16mm": (3.4, 20.9),
				"17mm": (3.6, 21.2),				
				"18mm": (3.8, 22),
				"19mm": (4, 23.2)})

		self.lab = np.zeros((len(sizes), 1, 2))
		self.toolSizes = []

		# loop over the dictionary
		for (i, (name, size)) in enumerate(sizes.items()):
			self.lab[i] = size
			self.toolSizes.append(name)

	def get_camera_info(self, msg):
		self.image_width = msg.width
		self.image_height = msg.height
		self.camera_K = msg.K

	def label(self, cntD):

		# initialize the minimum distance found thus far
		minDist = (np.inf, None)

		# loop over the known label values
		for (i, row) in enumerate(self.lab):
			d = distance.euclidean(row[0], cntD)

			# if the distance is smaller than the current distance,
			# then update the bookkeeping variable
			if d < minDist[0]:
				minDist = (d, i)

		# return the size label with the smallest distance
		return self.toolSizes[minDist[1]]


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

		WW=self.image_width
		HH=self.image_height

		WW=self.image_width
		HH=self.image_height

		fx=self.camera_K[0]
		fy=self.camera_K[4]
		u0=self.camera_K[5]
		v0=self.camera_K[2]

		K=np.matrix([[fx, 0, u0, 0], [0, fy, v0, 0], [0, 0, 1, 0]])
		K_INV=pinv(K)

		img = cv_image.copy()
		#img_org = cv2.imread('/home/mbzirc-01/Pictures/wrenches_blk_2.jpg')
		#img_org = cv2.imread('/home/mbzirc-01/Pictures/panel_query.JPG')
		#img = imutils.resize(img_org, width=640)

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

		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,12))
		filled = cv2.morphologyEx(edged4, cv2.MORPH_CLOSE, kernel)

		"""
		cv2.imshow("thresh", thresh)
		cv2.waitKey(10)

		cv2.imshow("edged4", edged4)
		cv2.waitKey(10)

		cv2.imshow("filled", filled)
		cv2.waitKey(10)
		"""

		# find contours in the thresholded image and initialize the shape detector
		#cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		#cnts = cv2.findContours(edged4.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cv2.findContours(filled.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]

		# sort the contours from left-to-right and initialize the
		(cnts, _) = contours.sort_contours(cnts)

		#print(cnts)

		#zlab = ToolLabeler()

		#=========================================================================
		"""		
		circles = cv2.HoughCircles(filled, cv2.cv.CV_HOUGH_GRADIENT,1.2,100,param1=150,param2=43,minRadius=WW/400,maxRadius=WW/30)
		#circles = cv2.HoughCircles(filled, cv2.cv.CV_HOUGH_GRADIENT,1.2,100)

		out4 = img.copy()

		# ensure at least some circles were found
		if circles is not None:
			# convert the (x, y) coordinates and radius of the circles to integers
			circles = np.round(circles[0, :]).astype("int")
		 
			# loop over the (x, y) coordinates and radius of the circles
			for (x, y, r) in circles:
				# draw the circle in the output image, then draw a rectangle
				# corresponding to the center of the circle
				cv2.circle(out4, (x, y), r, (0, 255, 0), 4)
				cv2.rectangle(out4, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
		 
			# show the output image
			cv2.imshow("out4", out4)
			cv2.waitKey(3)
		"""
		#===============================================================================


		# loop over the contours
		simil = []
		cX_v= []
		cY_v= [] 
		wr_cent_v= [] 
		wr_contours = []
		t_h_v = []
		wr_tc_v = []
		wrenches = []
		wr_count = 0
		toolIdentified = False
		for c in cnts:
			# compute the center of the contour, then detect the name of the
			# shape using only the contour
			M = cv2.moments(c)
			hu = cv2.HuMoments(M)

			#retSim = cv2.matchShapes(cnts_s[0],c,3,0.0)
			#simil = np.hstack((simil,retSim))

			retSim1 = cv2.matchShapes(cnts_s[0],c,1,0.0)
			retSim2 = cv2.matchShapes(cnts_s[0],c,2,0.0)
			retSim3 = cv2.matchShapes(cnts_s[0],c,3,0.0)

		 
			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours and the name of the shape on the image
			c = c.astype("float")
			c *= 1
			c = c.astype("int")
			text = "{}".format(shape)

			# if the contour is too large or too small, ignore it
			if cv2.contourArea(c) < 80 or cv2.contourArea(c) > 0.1*img.shape[0]*img.shape[1]:
				continue			

			# approximate the contour
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.01 * peri, True)
			(x, y, w, h) = cv2.boundingRect(approx)
			aspectRatio = w / float(h)

			(xc,yc),radius = cv2.minEnclosingCircle(c)
			minEncCirArea = math.pi*(radius**2)

			minEncircleA_ratio = minEncCirArea/cv2.contourArea(c) 
			
			#out3 = img.copy()
			#print(len(approx),aspectRatio,minEncircleA_ratio,minEncCirArea,cv2.contourArea(c),radius)
			#print(retSim1,retSim2,retSim3,aspectRatio)
		 
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
			#cv2.drawContours(out3, [box.astype("int")], -1, (0, 255, 0), 2)
			#print('hello',[box.astype("int")])
			(tl, tr, br, bl) = box

			#cv2.imshow("out3", out3)
			#cv2.waitKey(10)

			dA = distance.euclidean(tl, bl)
			dB = distance.euclidean(tl, tr)

			keepRatio = aspectRatio > 0.1 and aspectRatio < 0.5
			#keepSimilarity = retSim > 0.8 and retSim < 1.2
			#keepSimilarity = retSim1 < 18 and retSim1 > 10 and retSim2 > 0.8 and retSim3 > 0.9
			keepSimilarity = retSim1 < 60 and retSim1 > 8 and retSim2 > 0.7 and retSim3 > 0.9
			#keepSimilarity = retSim2 > 0.7 and retSim3 > 0.9

			if keepRatio and keepSimilarity:
				wr_count = wr_count + 1
				#wr_contours.append(c)
				wr_contours = np.append(wr_contours,c)
				#wr_contours = np.concatenate((wr_contours,c))

				cX = int((M["m10"] / M["m00"]))
				cY = int((M["m01"] / M["m00"]))

				cX_v = np.hstack((cX_v,cX))
				cY_v = np.hstack((cY_v,cY))
				
				wr_cent = (cX,cY)
				wr_cent_v = np.append(wr_cent_v,wr_cent)

				wrenches.append(c)				

				(t_x, t_y, t_w, t_h) = cv2.boundingRect(c)

				t_h_v = np.append(t_h_v,t_h)

				#wr_tc = (t_x,t_y)+t_w*0.5 				
				#wr_tc_v.append(wr_tc) 


			wr_cent_v = np.reshape(wr_cent_v,(-1,2))
			wr_cent_var = np.var(wr_cent_v,axis=0)			
			cX_var = np.var(cX_v)
			cY_var = np.var(cY_v)
			t_h_var = np.var(t_h_v)

			errPer = np.absolute(cY_var - t_h_var)/t_h_var
	

		if len(wr_contours) > 0:
			wr_contours = np.reshape(wr_contours,(1,-1,2))	
			wr_contours = wr_contours.astype(int)
			#print(wr_contours)
			(wrs_x, wrs_y, wrs_w, wrs_h) = cv2.boundingRect(wr_contours)
			cv2.rectangle(output, (wrs_x, wrs_y), (wrs_x + wrs_w, wrs_y + wrs_h), (255, 0, 0), 2)


			#=====================================================================================	
			wrs_box = cv2.minAreaRect(wr_contours)
			wrs_box = cv2.cv.BoxPoints(wrs_box) if imutils.is_cv2() else cv2.boxPoints(wrs_box)
			wrs_box = np.array(wrs_box, dtype="int")
			# order the points in the contour such that they appear
			# in top-left, top-right, bottom-right, and bottom-left
			# order, then draw the outline of the rotated bounding
			# box
			wrs_box = perspective.order_points(wrs_box)
			(wrs_tl, wrs_tr, wrs_br, wrs_bl) = wrs_box
			wrs_dH = distance.euclidean(wrs_tl, wrs_bl)
			wrs_dW = distance.euclidean(wrs_bl, wrs_br)

			incln_angle = math.degrees(math.asin(np.absolute((wrs_tl[0]-wrs_bl[0])/wrs_dH)))

			#print('angle',incln_angle)

			#cv2.drawContours(output, [wrs_box.astype("int")], -1, (0, 255, 0), 2)

			#=====================================================================================

			#print(errPer,wr_count)

			if errPer < 80 and  wr_count == 6:

				wrs_Z = fx*(act_wrs_w/float(wrs_dW))

				wrs_cX = wrs_x + wrs_w/2
				wrs_cY = wrs_y + wrs_h/2

				p_pxl_hom=np.matrix([[wrs_cY],[wrs_cX],[1]])
				P_mtr_hom=np.dot(K_INV,p_pxl_hom)
				P_mtr=P_mtr_hom*(wrs_Z/P_mtr_hom[2][0])

				#print(P_mtr)
				#print(wrenches)

				cv2.line(output, (WW/2,0), (WW/2,HH), (0, 0, 255), 1) 
				cv2.line(output, (0,HH/2), (WW,HH/2), (0, 0, 255), 1) 

				#result = WrenchDetectionResult()

				#result.ROI = [pnl_x, pnl_y, pnl_w, pnl_h]
				#self.server.set_succeeded(result)

				# Disable the node since it found its target
				#self.is_node_enabled = False

				"""
				#====================================================================
				#2D image points. If you change the image, you need to change vector
				image_points = np.array(wr_tc_v, dtype="double")
				 
				# 3D model points.
				model_points = np.array([
							    (-100, 0.0, 0.0),             # Nose tip
							    (-50, 0.0, 0.0),        # Chin
							    (0.0, 00.0, 0.0),     # Left eye left corner
							    (50.0, 0.0, 0.0),      # Right eye right corne
							    (100.0, 0.0, 0.0),    # Left Mouth corner
							    (150.0, 0.0, 0.0)      # Right mouth corner
							 
							])
				#====================================================================
				"""

				out3=img.copy()
				for wr in wrenches:
					(tool_px, tool_py, tool_pw, tool_ph) = cv2.boundingRect(wr)

					#=====================================================================================					
					tool_box = cv2.minAreaRect(wr)
					tool_box = cv2.cv.BoxPoints(tool_box) if imutils.is_cv2() else cv2.boxPoints(tool_box)
					tool_box = np.array(tool_box, dtype="int")
					# order the points in the contour such that they appear
					# in top-left, top-right, bottom-right, and bottom-left
					# order, then draw the outline of the rotated bounding
					# box
					tool_box = perspective.order_points(tool_box)
					(tool_tl, tool_tr, tool_br, tool_bl) = tool_box
					tool_dH = distance.euclidean(tool_tl, tool_bl)
					tool_dW = distance.euclidean(tool_bl, tool_br)
					#=====================================================================================

					#tool_width=tool_pw
					#tool_hight=tool_ph

					tool_width=tool_dW
					tool_hight=tool_dH
					
					tool_w = P_mtr[2][0]*tool_width/fx
					tool_h = P_mtr[2][0]*tool_hight/fx
					
					size = self.label(np.array([tool_w*100,tool_h*100]))
					#size = self.label(np.array([tool_h*100]))
					#print(size)

					
					#print(tool_pw,tool_ph,tool_dW,tool_dH,tool_w*100,tool_h*100,size)

										
					cv2.drawContours(out3, [tool_box.astype("int")], -1, (0, 255, 0), 2)
					cv2.imshow("out3", out3)
					cv2.waitKey(3)
					

					if size == self.tool_size:
						tool_contour = wr
						toolIdentified = True



		if toolIdentified:
			(tool_x, tool_y, tool_w, tool_h) = cv2.boundingRect(tool_contour)
			cv2.rectangle(output, (tool_x, tool_y), (tool_x + tool_w, tool_y + tool_h), (255, 0, 255), 2)

			self.tool_ROI.x_offset = tool_x
			self.tool_ROI.y_offset = tool_y
			self.tool_ROI.width = tool_w
			self.tool_ROI.height = tool_h

			self.tool_ROI_pub.publish(self.tool_ROI)

			tool_cX = tool_x + tool_w/2
			tool_cY = tool_y + tool_h/2

			tool_pxl_hom=np.matrix([[tool_cY],[tool_cX],[1]])
			tool_mtr_hom=np.dot(K_INV,tool_pxl_hom)
			tool_mtr=tool_mtr_hom*(wrs_Z/tool_mtr_hom[2][0])

			cv2.putText(output, "X={}".format(-tool_mtr[0][0]), (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
			cv2.putText(output, "Y={}".format(-tool_mtr[1][0]), (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
			cv2.putText(output, "Z={}".format(tool_mtr[2][0]), (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

			cv2.circle(output, (tool_cX, tool_cY), 3, (0, 0, 255), -1)

			self.tool_pos.pose.position.x = -tool_mtr[0][0]
			self.tool_pos.pose.position.y = -tool_mtr[1][0]
			self.tool_pos.pose.position.z = tool_mtr[2][0]

			self.tool_pos_pub.publish(self.tool_pos)
			
	

		# show the output image
		cv2.imshow("out2", output)
		cv2.waitKey(3)


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
