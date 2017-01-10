#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10/04/2016
@author: Tarek Taha

Naive code to perform static exploration

"""

import rospy
import math
from math import radians, degrees, cos, sin, tan, pi, atan2
import csv
import sys, os

import time
import cv2
import numpy as np

import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseWithCovariance, Twist, Point, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# Variables
lostMarker = False

laser_min_range = 1   # Used to ignore the UR5 arm
wall_distance = 2
closest_obstacle = 30

hough_angle_res = math.pi/32 # rad. Used for hough
hough_range_res = 0.05 # meters. Used for hough
hough_threshold = 20   # Minimum points to consider a line. Used for hough



class mbzirc_panel_track():
  def __init__(self):
    rospy.init_node('find_panel', anonymous=True)
    rospy.on_shutdown(self.shutdown)

    ############
    ## Variables
    ############
    self.desired_dist = 2.2

    self.ar_callback_start_time = rospy.Time()
    self.pose_callback_start_time = rospy.Time()
    self.foundMarker = False

    self.waypoints = list()
    self.quaternions = list()

    euler_angles = (0, -pi/2, pi, pi/2)
    for angle in euler_angles:
        q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
        q = Quaternion(*q_angle)
        self.quaternions.append(q)


    ############
    ## Set up subscribers, publishers and navigation stack
    ############
    self.current_pose = [0,0,0,0]
    #self.gotOdom = False
    ##self.scan_sub   = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=5)
    self.marker_sub = rospy.Subscriber('/visualization_marker', Marker, self.arCallback, queue_size=5)
    #self.pose_sub   = rospy.Subscriber("/odometry/filtered", Odometry, self.poseCallback)
    ##self.marker_pub = rospy.Publisher ("/explore/HoughLines", Marker, queue_size = 100)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    ## Subscribe to the move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")

    ## Wait 60 seconds for the action server to become available
    self.move_base.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to move base server")

    self.start()


  def shutdown(self):
    rospy.loginfo("Stopping the robot...")
    self.move_base.cancel_goal()
    rospy.sleep(2)
    self.cmd_vel_pub.publish(Twist())
    rospy.sleep(1)


  def start(self):
    ## Wait for initial odometry
    #while (not self.gotOdom and not rospy.is_shutdown()):
    #  time.sleep(0.01)

    ## Case 1: No object detected, move around
    ## Case 2: Object detected, go in front of it
    ##  Case 2.1: Oops, false alarm. Go back to moving
    ##  Case 2.2: Done positioning. Terminate
    ## Case 3: Went 360, couldn't find it. Go back to exploration

    ## 1: Move at slight angle perp to wall



    ## 2: Object detected
    while (not self.foundMarker and not rospy.is_shutdown()):
      time.sleep(0.1)


    rospy.loginfo("Detected marker. Setting goal")


    # Distances greater than dist_step_threshold result in us incrementally moving the robnot by 1m distances to account for odom drift
    dist_step_threshold = self.desired_dist + 1
    dist_goal = 1000000

    while ( dist_goal > dist_step_threshold and not rospy.is_shutdown() ):
      if (self.foundMarker):
        ## Sensor Coordinate frame: x = right, y = down, z = front
        ## Nav Coordinate frame:    x = front, y = left, z = up

        x_goal =   self.marker_position.z - self.desired_dist*cos(self.marker_angle)
        y_goal = -(self.marker_position.x - self.desired_dist*sin(self.marker_angle))
        dist_goal = math.sqrt(x_goal*x_goal + y_goal*y_goal)

        angle1 = atan2(y_goal, x_goal)
        angle2 = -angle1 - self.marker_angle

        print("x_d = " + str(round(x_goal,3)) + " y_d = " + str(round(y_goal,3)) + " a = " + str(round(degrees(self.marker_angle))) )


      # Rotate to target
        pose = [0, 0, 0, angle1]
        goal = self.generateRelativePositionGoal(pose)
        self.move( goal )

        if rospy.is_shutdown():
           return;

      # Drive to target
      m = dist_goal
      if (dist_goal > dist_step_threshold):
        m = 1

      pose = [m, 0, 0, 0]
      goal = self.generateRelativePositionGoal(pose)
      self.move( goal )

      #Wait a second to find marker
      self.foundMarker = False

      for i in range(0,10):
        if (not self.foundMarker and not rospy.is_shutdown()):
          time.sleep(0.1)

      if (not self.foundMarker):
        dist_goal -= 1



    if rospy.is_shutdown():
       return;

    # Rotate to final position
    if ( abs(angle2) > radians(10) ):
      pose = [0, 0, 0, angle2]
      goal = self.generateRelativePositionGoal(pose)
      self.move( goal )

    rospy.signal_shutdown("Complete")



  def move(self, goal):
    # Send the goal pose to the MoveBaseAction server
    self.move_base.send_goal(goal)

    # Allow 1 minute to get there
    finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

    # If we don't get there in time, abort the goal
    if not finished_within_time:
      self.move_base.cancel_goal()
      rospy.loginfo("Timed out achieving goal")
    else:
      ## We made it!
      state = self.move_base.get_state()
      #if state == GoalStatus.SUCCEEDED:
      rospy.loginfo("Goal succeeded!")


  def generateRelativePositionGoal(self, pose):
    # Compute pose
    x = pose[0]
    y = pose[1]
    z = pose[2]
    yaw = pose[3]
    quat = quaternion_from_euler(0.0, 0.0, yaw)

    # Intialize the waypoint goal
    goal = MoveBaseGoal()

    # Use the map frame to define goal poses
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose( Point(x, y, z) , Quaternion(*quat.tolist()) )
    return goal


  def arCallback(self, data):
    # Coordinate frame: x = right, y = down, z = front
    self.marker_position = data.pose.position
    roll, pitch, yaw = euler_from_quaternion([data.pose.orientation.x,
                                             data.pose.orientation.y,
                                             data.pose.orientation.z,
                                             data.pose.orientation.w])
    self.marker_angle = pitch
    self.foundMarker = True

    # Show message once a second
    if (rospy.Time.now() - self.ar_callback_start_time > rospy.Duration(1)):
      self.ar_callback_start_time = rospy.Time.now()
      print (self.marker_position)
      print ("yaw: " + str(degrees(self.marker_angle)) )

      print("")


  def poseCallback(self, data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
    self.current_pose = [x,y,z,yaw]
    self.gotOdom = True

    # Show message once a second
    if (rospy.Time.now() - self.pose_callback_start_time > rospy.Duration(1)):
      self.pose_callback_start_time = rospy.Time.now()
      rospy.loginfo("Current Location x: " + str(round(x,3)) + "\ty: " + str(round(y,3)) + "\tz: " + str(round(z,3)) + "\tyaw: " + str(round(degrees(yaw), 1)))


if __name__=='__main__':
  try:
      mbzirc_panel_track()
      rospy.spin()
  except rospy.ROSInterruptException:
      rospy.loginfo("mbzirc_c2_auto finished.")
