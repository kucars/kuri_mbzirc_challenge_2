#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10/04/2016
@author: Tarek Taha
Updated by: Dongming

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
from kuri_mbzirc_challenge_2.msg import HuskynavAction, HuskynavFeedback, HuskynavResult, HuskynavGoal

if __name__ == '__main__':
      rospy.init_node("husky_navigation_client")   
      client = actionlib.SimpleActionClient('husky_navigate', HuskynavAction)
      rospy.loginfo("Connecting to the Nav Action Server")
      client.wait_for_server()
      rospy.loginfo("Connected to Nav Action Server")
      goal = HuskynavGoal()
      client.send_goal(goal)
      rospy.loginfo("Goal sent")
      client.wait_for_result()
