#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10/04/2016
@author: Tarek Taha

Naive code to perform static exploration

"""

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees, cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

#[(15, 5, 0.0, 0.0)],
waypoints=[[(15, -10, 0.0, 0.0)],[(45, -10, 0.0, 0.0)],[(85, -10, 0.0, 0.0)]]

def generateGoal(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = '/odom'
    goal.target_pose.pose.position.x = pose[0][0]
    goal.target_pose.pose.position.y = pose[0][1]
    goal.target_pose.pose.position.z = pose[0][2]
   
    angle = pose[0][3]
    quat = quaternion_from_euler(0.0, 0.0, angle)
    goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
    
    return goal

def poseCallback(data):
    global current_pose
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
    current_pose = [x,y,z,yaw]
    #rospy.loginfo("Current Location x: " + str(x) + "y: " + str(y) + "z: " + str(z) + " yaw: " + str(degrees(yaw)))

#for laser
def scan_callback(msg):
        global g_range_ahead
        global obstacle_angle
        global angle_discrete
        global angle_number
        global angle_id
        #g_range_ahead = min(msg.ranges)
        min_range = 1000
        for i in range(len(msg.ranges)):
	  if msg.ranges[i] < min_range:
	    min_range = msg.ranges[i]
	    min_angle = msg.angle_max - 2 * i * msg.angle_max / len(msg.ranges)
	    angle_id = i
	g_range_ahead = min_range
	obstacle_angle = min_angle
	angle_discrete = msg.angle_increment
	angle_number = len(msg.ranges)
	
	
	  #if msg.ranges[i]==g_range_ahead:
	  # g_range_ahead_id=i
	  # obstacle_angle = msg.angle_min + 2 * g_range_ahead_id * msg.angle_max / len(msg.ranges)
	  # msg.angle_min + i * msg.angle_increment

if __name__=='__main__':
    rospy.init_node("find_panel")   
    
    rospy.Subscriber("/odometry/filtered", Odometry)
    
    navigationActionServer = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    #for x in xrange(16, 116):
    #    for y in xrange(-4,54):
    #        print x,y
            
    rospy.loginfo("Connecting to the move Action Server")
    navigationActionServer.wait_for_server()
    rospy.loginfo("Ready ...")
    
    #for laser
    g_range_ahead = 30 # anything to start
    current_pose = [0,0,0,0]
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    pose_sub = rospy.Subscriber("/odometry/filtered", Odometry, poseCallback)
    
    print "current_pose: ", str(current_pose)
    
    #while True:
    for pose in waypoints:
	rospy.loginfo("Creating navigation goal...")
	goal = generateGoal(pose)
	
	rospy.loginfo("Moving Robot desired goal")
	navigationActionServer.send_goal(goal)
	
	#to stop if obstacle is sensed in the range of laser
	while (navigationActionServer.get_state()==0 or navigationActionServer.get_state()==1):
	  rospy.sleep(0.1)
	  if (g_range_ahead < 29):
	    navigationActionServer.cancel_goal()
	    
	    rospy.loginfo("Obstacle in front")
	    break
	
	#to break out from the waypoints loop
	if (g_range_ahead < 29):
	  break
	
	rospy.loginfo("Waiting for Robot to reach goal")
	navigationActionServer.wait_for_result()
	
	rospy.sleep(10.)
    
    #define the obstacle moving goal
    rospy.loginfo("Creating obstacle goal...")
    print "current_pose: ", str(current_pose)
    print "obstacle_angle", str(obstacle_angle)
    print "angle_discrete", str(angle_discrete)
    print "angle_id", str(angle_id)
    print "angle_number", str(angle_number)
    
    obst_goal_local_position = [(g_range_ahead - 2) * cos(obstacle_angle),(g_range_ahead - 2) * sin(obstacle_angle)]
    obst_goal_global_position = [current_pose[0] + cos(current_pose[3]) * obst_goal_local_position[0] - sin(current_pose[3]) * obst_goal_local_position[1], current_pose[1] + sin(current_pose[3]) * obst_goal_local_position[0] + cos(current_pose[3]) * obst_goal_local_position[1]]
    pose = [(obst_goal_global_position[0],obst_goal_global_position[1], 0, obstacle_angle + current_pose[3])]
    goal = generateGoal(pose)
    
    print "cos(current_pose[3])", str(cos(current_pose[3]))
    print "local postion: ", str(obst_goal_local_position)
    print "global postion: ", str(obst_goal_global_position)
    print "goal: ", str(pose)
	
    rospy.loginfo("Moving Robot to the obstacle")
    navigationActionServer.send_goal(goal)
    
    	#to stop 3 meters away of the obstacle
    while (navigationActionServer.get_state()==0 or navigationActionServer.get_state()==1):
	  rospy.sleep(0.1)
	  if (g_range_ahead < 3):
	    navigationActionServer.cancel_goal()
	    
	    rospy.loginfo("3 meter in front of Obstacle")
	    break
    
    rospy.loginfo("Waiting for Robot to reach obstacle goal")
    navigationActionServer.wait_for_result()    
    
    rospy.loginfo("Generate the desired configuration in front of panel")
    pose = [(64, -25, 0.0, 0.0)]
    goal = generateGoal(pose)
    rospy.loginfo("Moving Robot to the desired configuration in front of panel")
    navigationActionServer.send_goal(goal)
    rospy.loginfo("Waiting for Robot to reach the desired configuration in front of panel")
    navigationActionServer.wait_for_result()  
    
    navResult  = navigationActionServer.get_result()
    navState   = navigationActionServer.get_state()

    print "g_range_ahead: ", str(g_range_ahead)
 
    rospy.loginfo("Finished Navigating")
    print "Result: ", str(navResult)
    # Outcome 3 : SUCCESS, 4 : ABORTED , 5: REJECTED
    print "Navigation status: ", str(navState)

    
    
 
