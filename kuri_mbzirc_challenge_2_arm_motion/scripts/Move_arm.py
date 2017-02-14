#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import sys
from control_msgs.msg import *
from kuri_mbzirc_challenge_2_msgs.msg import ArmMotionAction
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

node_name = "arm_motion_server"
topic_name = "arm_motion"


# Define the set of Joints
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Set the joint angles in Degrees
D1 = [-53.58,-78.67,72.29,-172.56,-35.39,0.73]
D2 = [-76.42,-95.31,90.07,-171.25,-12.54,-1.46]
D3 = [-53.58,-78.67,72.29,-172.56,-35.39,0.73]
D4 = [-76.42,-95.31,90.07,-171.25,-12.54,-1.46]
D5 = [-53.58,-78.67,72.29,-172.56,-35.39,0.73]
D6 = [-76.42,-95.31,90.07,-171.25,-12.54,-1.46]

# Convert the joint angles to Radians
Q1 = [D1[i]*pi/180 for i in xrange(0,6)]
Q2 = [D2[i]*pi/180 for i in xrange(0,6)]
Q3 = [D3[i]*pi/180 for i in xrange(0,6)]
Q4 = [D4[i]*pi/180 for i in xrange(0,6)]
Q5 = [D5[i]*pi/180 for i in xrange(0,6)]
Q6 = [D6[i]*pi/180 for i in xrange(0,6)]

# Set the joint angles in Radians
#Q1 = [-1.3337806,-1.6634733,1.5720181,-2.98887634,-0.21886429,-0.02548181]
#Q2 = [-0.04101524,-2.08723925,1.75859375,-2.77716791,-1.375494,-0.001919862]
#Q3 = [-1.3337806,-1.6634733,1.5720181,-2.98887634,-0.21886429,-0.02548181]
#Q4 = [-0.04101524,-2.08723925,1.75859375,-2.77716791,-1.375494,-0.001919862]
#Q5 = [-1.3337806,-1.6634733,1.5720181,-2.98887634,-0.21886429,-0.02548181]
#Q6 = [-0.04101524,-2.08723925,1.75859375,-2.77716791,-1.375494,-0.001919862]


class ArmMotionServer:
    # Variables
    client = None

    def __init__(self, is_bypass):
        ##########
        ## Initialize state machine interface
        ##########
        self.is_bypass = is_bypass

        self.is_node_enabled = False
        if (self.is_bypass):
            self.is_node_enabled = True

        # Start actionlib server
        self.server = actionlib.SimpleActionServer(topic_name, ArmMotionAction, self.execute, False)
        self.server.start()

        rospy.loginfo("Started " + node_name + " node. Currently on standby")

        ##########
        ## Initialize node
        ##########
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"

        if (self.is_node_enabled):
          self.execute(None)


    def execute(self, goal_msg):
        # This node was called, perform any necessary changes here
        self.is_node_enabled = True
        rospy.loginfo(node_name + " node enabled")

        self.main() # Execute


    def main(self):
        try:
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(JOINT_NAMES):
                    JOINT_NAMES[i] = prefix + name
            print "This program makes the robot move between the following poses:"
            print str([Q1[i]*180./pi for i in xrange(0,6)])
            print str([Q2[i]*180./pi for i in xrange(0,6)])
            print str([Q3[i]*180./pi for i in xrange(0,6)])
            print str([Q4[i]*180./pi for i in xrange(0,6)])
            print str([Q5[i]*180./pi for i in xrange(0,6)])
            print str([Q6[i]*180./pi for i in xrange(0,6)])
            print "Please make sure that your robot can move freely between these poses before proceeding!"
            inp = raw_input("Continue? y/n: ")[0]
            if (inp == 'y'):
                self.move1()
            else:
                print "Halting program"
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise


    # Define the First function: Move to specific positions (once)
    def move1(self):
        # Define the type of the message
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            # Read topic joint states, take only position (angles in Rad), save them in a global variable (joints_pos)
            self.joints_pos = joint_states.position
            # Current state is the first way point, Move to these wayepoints, Duration determines the speed, Changing velocities will move with a certain velocity for 5 seconds
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(4.0)),
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(6.0)),
                JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(8.0)),
                JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(10.0)),
                JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(12.0))]
            # Final duration means "the trajectory takes this time to finish"
            # This client sends the designed trajectory
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

if __name__ == '__main__':
    rospy.init_node(node_name)

    is_bypass = False
    if len(sys.argv) >= 2:
        is_bypass = True

    server = ArmMotionServer(is_bypass)
    rospy.spin()
