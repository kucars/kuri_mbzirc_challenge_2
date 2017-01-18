#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time

import actionlib

from kuri_mbzirc_challenge_2_msgs.msg import PanelPositionAction, HuskynavAction, BoxPositionAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# The code is only for state machine overview. To plug in the actual code for each state, please comment out the definition in the code and include the actual code.

# Orson Lin 10.09.2016
sleep_time = 0.3


# define state : initialization
class initialization(smach.State):
    def __init__(self,test_num):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['init_params'],
                             output_keys=['init_out'])
        self.test_num=test_num
    def execute(self, userdata):
        print self.test_num
        rospy.loginfo('Executing state INITIALIZATION')
        userdata.init_out = userdata.init_params + 1
        time.sleep(sleep_time)
        return 'succeeded'


######
## POSITIONING
######

class move_cluster_search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])


    def execute(self, userdata):
        time.sleep(sleep_time)
        return 'succeeded'


######
## WRENCH
######

# define state : detect_wrench
class detect_wrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['detect_wrench_in'],
                             output_keys=['detect_wrench_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DETECT_WRENCH')
        userdata.detect_wrench_out = userdata.detect_wrench_in + 1
        time.sleep(sleep_time)
        return 'succeeded'
 # define state : detect_valve
class detect_valve(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             #input_keys=['detect_valve_in'],
                             output_keys=['detect_valve_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DETECT_VALVE')
        userdata.detect_valve_out = 1 #userdata.detect_valve_in + 1
        time.sleep(sleep_time)
        return 'succeeded'

# define state : grip_pose_calculation
class grip_pose_calculation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['grip_pose_calculation_in'],
                             output_keys=['grip_pose_calculation_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GRIP_POSE_CALCULATION')
        userdata.grip_pose_calculation_out = userdata.grip_pose_calculation_in + 1
        time.sleep(sleep_time)
        return 'succeeded'

# define state : pick_wrench
class pick_wrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['pick_wrench_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PICK_WRENCH')
        #userdata.grip_pose_calculation_out = userdata.grip_pose_calculation_in + 1
        time.sleep(sleep_time)
        return 'succeeded'

# define state : move_to_valve
class move_to_valve(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['move_to_valve_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_TO_VALVE')
        time.sleep(sleep_time)
        return 'succeeded'
# define state : operate_valve
class operate_valve(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['operate_valve_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OPERATE_VALVE')
        time.sleep(sleep_time)
        return 'succeeded'






# main
class Challenge2():
    def __init__(self):
        # Initialize a number of parameters and variables
        self.con_input=100000;
        self.con_output=0;
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['Done', 'aborted'])
        self.sm.userdata.user_input = 0


        ## DETECTION
        self.sm_detection = smach.Concurrence(
                              outcomes=['succeeded', 'failed'],
                              default_outcome='succeeded',
                              child_termination_cb=self.sm_detection_con_termination,
                              #outcome_cb=self.concurrence_outcome_cb
                              )

        self.circumnavigating=smach.StateMachine(outcomes=['terminated', 'failed', 'preempted'])
        with self.circumnavigating:

            smach.StateMachine.add('GET_PANEL_CLUSTER',
                            smach_ros.SimpleActionState(
                                'get_panel_cluster', PanelPositionAction,
                                result_cb = self.get_panel_cluster_result_cb,
                                output_keys = ['waypoints']
                            ),
                            transitions={'aborted':'MOVE_CLUSTER_SEARCH','succeeded':'MOVE_PANEL_WAYPOINTS', 'preempted':'preempted'},
                            remapping={'waypoints':'cluster_waypoints'}
                            )

            smach.StateMachine.add('MOVE_CLUSTER_SEARCH', move_cluster_search(),
                            transitions={'succeeded':'GET_PANEL_CLUSTER'})

            smach.StateMachine.add('MOVE_PANEL_WAYPOINTS',
                            smach_ros.SimpleActionState(
                                'husky_navigate', HuskynavAction,
                                goal_slots=['waypoints']
                            ),
                            transitions={'aborted':'failed','succeeded':'terminated'},
                            remapping={'waypoints':'cluster_waypoints'}
                            )


        self.detect_panel=smach.StateMachine(outcomes=['terminated', 'failed', 'preempted'],
                                             output_keys=['panel_waypoint'])
        with self.detect_panel:
            # Add states to the container
            smach.StateMachine.add('DETECTING_PANEL',
                            smach_ros.SimpleActionState(
                                'panel_waypoint', PanelPositionAction,
                                result_cb = self.detecting_panel_result_cb,
                                output_keys = ['waypoints']
                            ),
                            transitions={'aborted':'DETECTING_PANEL',
                                         'succeeded':'terminated',
                                         'preempted': 'preempted'},
                            remapping={'waypoints':'panel_waypoint'}
                            )


        with self.sm_detection:
            smach.Concurrence.add('CIRCUMNAVIGATING', self.circumnavigating)
            smach.Concurrence.add('DETECTING_PANEL', self.detect_panel)

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('INITIATING', initialization(500000),
                            transitions={'succeeded':'EXPLORATION'},
                            remapping={'init_params':'user_input',
                                       'init_out':'start_info'})

            smach.StateMachine.add('EXPLORATION',
                            smach_ros.SimpleActionState(
                                'get_box_cluster', BoxPositionAction,
                                result_cb = self.get_panel_cluster_result_cb,
                                output_keys = ['waypoints']
                            ),
                            transitions={'succeeded':'MOVE_TO_BOX', 'preempted':'aborted'},
                            remapping={'waypoints':'box_waypoint'}
                            )


            smach.StateMachine.add('MOVE_TO_BOX',
                            smach_ros.SimpleActionState(
                                'husky_navigate', HuskynavAction,
                                goal_slots=['waypoints']
                            ),
                            transitions={'succeeded':'POSITIONING_IN_FRONT', 'preempted':'aborted'},
                            remapping={'waypoints':'box_waypoint'}
                            )

            smach.StateMachine.add('POSITIONING_IN_FRONT',
                            self.sm_detection,
                            transitions={'succeeded':'MOVE_IN_FRONT_PANEL',
                                        'failed':'EXPLORATION'}
                            )

            smach.StateMachine.add('MOVE_IN_FRONT_PANEL',
                            smach_ros.SimpleActionState(
                                'husky_navigate', HuskynavAction,
                                goal_slots=['waypoints']
                            ),
                            transitions={'aborted':'POSITIONING_IN_FRONT','succeeded':'DETECTING_VALVE','preempted':'aborted'},
                            remapping={'waypoints':'panel_waypoint'}
                            )

            smach.StateMachine.add('DETECTING_VALVE', detect_valve(),
                           transitions={'succeeded':'DETECTING_WRENCH'},
                           remapping={#'detect_valve_in':'panel_pose',
                                      'detect_valve_out':'valve_pose'})
            smach.StateMachine.add('DETECTING_WRENCH', detect_wrench(),
                           transitions={'succeeded':'CALCULATING_GRIP_POSE'},
                           remapping={'detect_wrench_in':'valve_pose',
                                      'detect_wrench_out':'wrench_pose'})
            smach.StateMachine.add('CALCULATING_GRIP_POSE', grip_pose_calculation(),
                           transitions={'succeeded':'PICKING_WRENCH'},
                           remapping={'grip_pose_calculation_in':'wrench_pose',
                                      'grip_pose_calculation_out':'grip_pose'})
            smach.StateMachine.add('PICKING_WRENCH', pick_wrench(),
                           transitions={'succeeded':'MOVING_TO_VALVE'},
                           remapping={'pick_wrench_in':'grip_pose'})
            smach.StateMachine.add('MOVING_TO_VALVE', move_to_valve(),
                           transitions={'succeeded':'OPERATING_VALVE'},
                           remapping={'move_to_valve_in':'valve_pose'})
            smach.StateMachine.add('OPERATING_VALVE', operate_valve(),
                           transitions={'succeeded':'Done'},
                           remapping={'operate_valve_in':'valve_pose'})


        sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = self.sm.execute()
        print self.con_output
        rospy.spin()
        sis.stop()


    # Define callback function for the concuerrent container
    # Gets called when ANY child state terminates
    def sm_exploration_con_termination(self, outcome_map):
        # If the current navigation task has succeeded, return True
        print 'Exploration termination'
        if outcome_map['EXPLORING'] == 'terminated':
            print 'preempt all the rest'
            return True
        else:
            return False

    # Gets called when ALL child states are terminated
    def concurrence_outcome_cb(self, outcome_map):
        print 'concurrent_terminated'
        print self.con_input
        print self.con_output
        self.con_output=self.con_input+1
        print self.con_output


        rospy.loginfo('Existing state CON')
        time.sleep(sleep_time)
        return 'succeeded'




    # Define callback function for the concuerrent container
    # Gets called when ANY child state terminates
    def sm_detection_con_termination(self, outcome_map):
        # If the current navigation task has succeeded, return True
        print 'Detection termination'
        if outcome_map['DETECTING_PANEL'] == 'terminated':
            print 'preempt all the rest'
            return True
        else:
            return False



    ## DETECT_PANEL
    def detect_panel_result_cb(self, userdata, status, result):
        if (result.success):
            userdata.waypoints = result.waypoints;
            print("Found the panel!")
            return 'succeeded'
        else:
            return 'aborted'

    ## GET_PANEL_CLUSTER
    def get_panel_cluster_result_cb(self, userdata, status, result):
        if (result.success):
            userdata.waypoints = result.waypoints;
            print("Number of waypoints: " + str(len(result.waypoints.poses)) )
            return 'succeeded'
        else:
            return 'aborted'

    ## DETECTING_PANEL
    def detecting_panel_result_cb(self, userdata, status, result):
        if (result.success):
            userdata.waypoints = result.waypoints;
            print("Number of waypoints: " + str(len(result.waypoints.poses)) )

            # This hack seems to be the only way I can get the user data out of this concurrency state
            self.sm.userdata.panel_waypoint = result.waypoints;

            return 'succeeded'
        else:
            return 'aborted'






if __name__ == '__main__':
    rospy.init_node('MBZIRC_ch2_state_machine')
    Challenge2()
