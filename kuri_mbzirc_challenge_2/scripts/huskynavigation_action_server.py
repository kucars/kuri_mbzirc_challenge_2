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
from kuri_mbzirc_challenge_2_msgs.msg import HuskynavAction, HuskynavResult

class Huskynavigationserver:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('husky_navigate', HuskynavAction, self.execute, False)
        self.server.start()
        self.g_range_ahead = 0
        self.current_pose = [0,0,0,0]

        self.navigationActionServer = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Connecting to the move Action Server")
        self.navigationActionServer.wait_for_server()
        rospy.loginfo("Ready ...")

    def execute(self, goal_msg):
        for w in goal_msg.waypoints.poses:
            rospy.loginfo("Executing waypoint")
            goal = self.generateGoal(goal_msg.waypoints.header.frame_id, w)
            self.navigationActionServer.send_goal(goal)

            self.navigationActionServer.wait_for_result()

            navResult  = self.navigationActionServer.get_result()
            navState   = self.navigationActionServer.get_state()

            """
            if navResult == 3:
                result.success = True
                rospy.loginfo("Husky navigation Succeeded")
                server.set_succeeded(result)
            """

        result = HuskynavResult()
        result.success = True
        rospy.loginfo("Husky navigation Succeeded")
        self.server.set_succeeded(result)

    def generateGoal(self, frame, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.pose = pose

        return goal

if __name__ == '__main__':
      rospy.init_node("husky_navigation_server")   
      server = Huskynavigationserver()
      rospy.spin()
