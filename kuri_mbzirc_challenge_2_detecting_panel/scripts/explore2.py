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
marker_position = []
marker_angle = 0

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




  def start(self):
    ## Wait for initial odometry
    #while (not self.gotOdom and not rospy.is_shutdown()):
    #  time.sleep(0.01)

    # Case 1: No object detected, move around
    # Case 2: Object detected, go in front of it
      # Case 2.1: Oops, false alarm. Go back to moving
      # Case 2.2: Done positioning. Terminate
    # Case 3: Went 360, couldn't find it. Go back to exploration

    # 1: Move at slight angle perp to wall



    # 2: Object detected
    while (not self.foundMarker and not rospy.is_shutdown()):
      time.sleep(0.1)

    #self.starting_pose = current_pose
    #self.starting_angle = 0
    self.desired_dist = 1.5

    rospy.loginfo("Detected marker. Setting goal")


    # Set waypoint
    goal = self.generateRelativePositionGoal([1, 0, 0, 0])
    self.waypoints.append( goal )

    #self.waypoints.append(Pose(Point(-1, 0, 0.0), self.quaternions[0]))

    # Cycle through the waypoints
    for i in range(0, len(self.waypoints) ):
      if rospy.is_shutdown():
        break;

      # Start the robot moving toward the goal
      self.move( self.waypoints[i] )

    rospy.signal_shutdown("Complete")

    """
    ## Sensor Coordinate frame: x = right, y = down, z = front
    ## Nav Coordinate frame:    x = front, y = left, z = up

    x_desired =   marker_position.z - desired_dist*cos(marker_angle)
    y_desired = -(marker_position.x - desired_dist*sin(marker_angle))
    dist_desired = math.sqrt(x_desired*x_desired + y_desired*y_desired)

    angle1 = atan2(y_desired, x_desired)

    print("x_d = " + str(round(x_desired,3)) + " y_d = " + str(round(y_desired,3)) + " a = " + str(round(degrees(marker_angle))) )
    pose = [x_desired, y_desired, 0, angle1]

    goal = generateRelativePositionGoalAdjusted(pose)




    # Drive to target
    pose = [x_desired, y_desired, 0, angle1]
    goal = generateRelativePositionGoalAdjusted(pose)
    rospy.loginfo("Moving Robot to the desired configuration in front of panel")
    navigationActionServer.send_goal(goal)
    rospy.loginfo("Waiting for Robot to reach the desired configuration in front of panel")
    navigationActionServer.wait_for_result()

    navResult  = navigationActionServer.get_result()
    navState   = navigationActionServer.get_state()

    # Rotate to final position
    pose = [0, 0, 0, -pi/2]
    goal = generateRelativePositionGoalAdjusted(pose)
    rospy.loginfo("Moving Robot to the desired configuration in front of panel")
    navigationActionServer.send_goal(goal)
    rospy.loginfo("Waiting for Robot to reach the desired configuration in front of panel")
    navigationActionServer.wait_for_result()

    navResult  = navigationActionServer.get_result()
    navState   = navigationActionServer.get_state()
    """

  def shutdown(self):
    rospy.loginfo("Stopping the robot...")
    self.move_base.cancel_goal()
    rospy.sleep(2)
    self.cmd_vel_pub.publish(Twist())
    rospy.sleep(1)



  def move(self, goal):
    # Send the goal pose to the MoveBaseAction server
    self.move_base.send_goal(goal)

    # Allow 1 minute to get there
    finished_within_time = self.move_base.wait_for_result(rospy.Duration(600))

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
      print (degrees(self.marker_angle))
      print (self.marker_position)
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









# Generate range using floats
def frange(start, end, jump):
  out = [start]
  while True:
    start += jump
    out.append(start)
    
    if (start > end):
      break;
    
  return out
    
def NormalizeAngle(a):
  # Get angle between (-pi, pi]
  if (a > pi):
    a -= 2*pi
  if (a <= -pi):
    a += 2*pi
    
  return a
    
class HoughLine:
  def __init__(self, r, angle, votes):
    self.r = r
    self.angle = angle
    self.votes = votes
  def __repr__(self):
    return repr((self.r, self.angle, self.votes))

# Hough transform designed for radial laser data
def houghTransform(r, alpha):
  global laser_min_range, hough_angle_res, hough_range_res, hough_threshold
  
  theta = frange(0, 2*math.pi, hough_angle_res)
  rho   = frange(0, max(r), hough_range_res)
  
  # Create 2D grid of theta and rho
  votes = [[0 for i in range(len(rho))] for j in range(len(theta))]
  
  # Analyze each point
  for p in range(len(r)):
    if (r[p] < laser_min_range):
      continue;
    
    for i_t in range(len(theta)):
      r_temp = r[p]*cos(alpha[p] - theta[i_t])
      if (r_temp < 0):
	continue
      
      r_temp = hough_range_res*round(r_temp/hough_range_res) # round to nearest value of rho
      i_r = int(r_temp/hough_range_res) # find index of corresponding rho
      
      votes[i_t][i_r] += 1
	
  # Extract lines by thresholding
  line_out = [] #Output lines
  for i_r in range(len(rho)):
    for i_t in range(len(theta)):
      if (votes[i_t][i_r] >= hough_threshold): # and votes[i_t][i_r] >= v_max*0.5
	h = HoughLine(rho[i_r], theta[i_t], votes[i_t][i_r])
	line_out.append(h)
  
  if (len(line_out) == 0):
    return
  
  # Sort them in descending order
  line_out = sorted(line_out, key=lambda l: l.votes, reverse=True)
  
  # Record the top angle for each +- 45 degree increment
  line_temp = []
  line_temp.append(line_out[0])
  for i in range(len(line_out)):
    if (line_out[i].r >= 20):
      continue
    
    # Record it if there are no similar recorded angles
    isValid = True
    for j in range (len(line_temp)):
      if ( abs(NormalizeAngle(line_out[i].angle - line_temp[j].angle) <= pi/4) ):
	isValid = False
	break
    if (isValid):
      line_temp.append(line_out[i])
      
  line_out = line_temp
  
  # Create markers for each
  marker = Marker()
  marker.header.frame_id = "/base_link"

  marker.type = marker.LINE_LIST
  marker.pose.orientation.w = 1

  marker.scale.x = 0.01
  marker.color.a = 1.0
  marker.color.b = 1.0
    
  # Print y = mx+c versions of each line and make a marker for each
  line_offset = -0.55
  for i in range(len(line_out)):
    m = tan(line_out[i].angle)
    c = line_out[i].r / cos(line_out[i].angle)
    q = 30
    y1, x1 = [-q, -m*q - c + line_offset]
    y2, x2 = [ q,  m*q - c + line_offset]
    
    #print ("votes = " + str(line_out[i].votes) + "\tr = " + str(line_out[i].r) + "\tangle = " + str(round(line_out[i].angle/pi*180))+ "\ty = " + str(round(m,3)) + "x + " + str(round(c,3)) )
    
    p1 = Point() 
    p1.x = -x1
    p1.y = y1
    p1.z = 0
    marker.points.append(p1)
    
    p2 = Point() 
    p2.x = -x2
    p2.y = y2
    p2.z = 0
    marker.points.append(p2)

  # Publish markers
  marker_pub.publish(marker)






# Laser scanner callback
def scan_callback(msg):
  global laser_min_range, closest_obstacle
  
  ranges = []
  angles = []
  
  closest_obstacle = 100
  for i in range(len(msg.ranges)):
    # Skip points at infinity or too close
    if ( math.isinf(msg.ranges[i]) or msg.ranges[i] < laser_min_range ):
      continue
    
    # Record range and compute angle
    ranges.append(msg.ranges[i])
    angles.append(msg.angle_max - 2 * i * msg.angle_max / len(msg.ranges))
    
    # Find closest obstacle
    if (msg.ranges[i] < closest_obstacle):
      closest_obstacle = msg.ranges[i]
  
  # Extract lines with hough transform
  houghTransform(ranges, angles)


if __name__=='__main__':
  try:
      mbzirc_panel_track()
      rospy.spin()
  except rospy.ROSInterruptException:
      rospy.loginfo("mbzirc_c2_auto finished.")
