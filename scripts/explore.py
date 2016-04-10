#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10/04/2016
@author: Tarek Taha

Naive code to perform static exploration

"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

def generateGoal(x, y, z,  yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = '/odom'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
   
    angle = radians(yaw)
    quat = quaternion_from_euler(0.0, 0.0, angle)
    goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
    
    return goal

def poseCallback(data):

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
    rospy.loginfo("Current Location x: " + str(x) + "y: " + str(y) + "z: " + str(z) + " yaw: " + str(degrees(yaw)))

if __name__=='__main__':
    rospy.init_node("find_panel")   
    
    rospy.Subscriber("/odometry/filtered", Odometry, poseCallback)
    
    navigationActionServer = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    #for x in xrange(16, 116):
    #    for y in xrange(-4,54):
    #        print x,y
            
    rospy.loginfo("Connecting to the move Action Server")
    navigationActionServer.wait_for_server()
    rospy.loginfo("Ready ...")
    
    rospy.loginfo("Creating navigation goal...")
    goal = generateGoal(15,5, 0.0, 0.0)
    
    rospy.loginfo("Moving Robot desired goal")
    navigationActionServer.send_goal(goal)
    
    rospy.loginfo("Waiting for Robot to reach goal")
    navigationActionServer.wait_for_result()
    
    navResult  = navigationActionServer.get_result()
    navState   = navigationActionServer.get_state()
    
    rospy.loginfo("Finished Navigating")
    print "Result: ", str(navResult)
    # Outcome 3 : SUCCESS, 4 : ABORTED , 5: REJECTED
    print "Navigation status: ", str(navState)

    
    
 
