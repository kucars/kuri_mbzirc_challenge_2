#!/usr/bin/env python

import roslib;
#import roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import time

from kuri_mbzirc_challenge_2_msgs.msg import PanelPositionAction

# The code is only for state machine overview. To plug in the actual code for each state, please comment out the definition in the code and include the actual code.

# Orson Lin 10.09.2016


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
        time.sleep(1)
        return 'succeeded'

######
## EXPLORATION
######

# define state : panel_searching
class panel_searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found','not_found'])
        self.search_time=0


    def execute(self, userdata):
        time.sleep(1)
        if self.search_time<3:
            rospy.loginfo('Target_not_found')
            self.search_time+=1
            print 'search result:'
            print  self.search_time
            return 'not_found'
        else:
            return 'found'





# define state : exploration_planning
class exploration_planning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_to_waypoint'])


    def execute(self, userdata):
        time.sleep(1)
        return 'move_to_waypoint'



# define state : move_base
class move_base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['evaluate_terminal_condition'])


    def execute(self, userdata):
        time.sleep(1)
        if self.preempt_requested():
                self.service_preempt()
                #return 'preempted'
        else:
                return 'evaluate_terminal_condition'

# define state : update_map
class update_map(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['updated','preempted'])

    def execute(self, userdata):
        time.sleep(1)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        else:
            return 'updated'


# define state : search
class search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.search_time=0
    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH')

        time.sleep(2)
        #return 'succeeded'


        if self.search_time<3:
            rospy.loginfo('Target_not_found')
            self.search_time+=1
            print 'search result:'
            print  self.search_time
            return 'failed'
        else:
            return 'succeeded'


######
## POSITIONING
######

class move_in_front_panel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])


    def execute(self, userdata):
        time.sleep(1)
        return 'succeeded'

class detect_panel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'preempted'],
                                   output_keys=['position_infront_out'])


    def execute(self, userdata):
        time.sleep(1)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        userdata.position_infront_out = 1
        return 'not_found'

class move_panel_waypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found'])


    def execute(self, userdata):
        time.sleep(1)
        return 'not_found'

class move_cluster_search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])


    def execute(self, userdata):
        time.sleep(1)
        return 'succeeded'


class get_panel_cluster(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'])


    def execute(self, userdata):
        time.sleep(1)
        return 'found'


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
        time.sleep(1)
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
        time.sleep(1)
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
        time.sleep(1)
        return 'succeeded'

# define state : pick_wrench
class pick_wrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['pick_wrench_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PICK_WRENCH')
        #userdata.grip_pose_calculation_out = userdata.grip_pose_calculation_in + 1
        time.sleep(1)
        return 'succeeded'

# define state : move_to_valve
class move_to_valve(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['move_to_valve_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_TO_VALVE')
        time.sleep(1)
        return 'succeeded'
# define state : operate_valve
class operate_valve(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['operate_valve_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OPERATE_VALVE')
        time.sleep(1)
        return 'succeeded'






# main
class Challenge2():
    def __init__(self):
        rospy.init_node('MBZIRC_ch2_state_machine')
        # Initialize a number of parameters and variables
        self.con_input=100000;
        self.con_output=0;
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['Done'])
        self.sm.userdata.user_input = 0


        ## Search
        self.sm_search = smach.Concurrence(
                              outcomes=['succeeded'],
                              default_outcome='succeeded',
                              child_termination_cb=self.sm_search_con_termination,
                              outcome_cb=self.concurrence_outcome_cb)

        self.exploring=smach.StateMachine(outcomes=['terminated'])
        with self.exploring:
            # Add states to the container
            smach.StateMachine.add('SEARCHING_PANEL_IN_MAP', panel_searching(),
                            transitions={'not_found':'EXPLORATION_PLANNING','found':'terminated'})
            # Add states to the container
            smach.StateMachine.add('EXPLORATION_PLANNING', exploration_planning(),
                            transitions={'move_to_waypoint':'MOVE_BASE'})
            # add the concurrent to the main state machine
            smach.StateMachine.add('MOVE_BASE', move_base(),
                            transitions={'evaluate_terminal_condition':'SEARCHING_PANEL_IN_MAP'})



        self.update_map=smach.StateMachine(outcomes=['preempted'])
        with self.update_map:
        # Add states to the container
            smach.StateMachine.add('UPDATING_MAP', update_map(),
                            transitions={'updated':'UPDATING_MAP'})

        with self.sm_search:
            smach.Concurrence.add('EXPLORING', self.exploring)
            smach.Concurrence.add('UPDATING_MAP', self.update_map)


        ## POSITION
        self.sm_positioning = smach.Concurrence(
                              outcomes=['succeeded', 'failed'],
                              default_outcome='succeeded',
                              child_termination_cb=self.sm_positioning_con_termination,
                              #outcome_cb=self.concurrence_outcome_cb
                              )

        def result_cb(user_data, status, result):
            if (result.success):
                return 'succeeded'
            else:
                return 'aborted'


        self.circumnavigating=smach.StateMachine(outcomes=['terminated', 'failed', 'preempted'])
        with self.circumnavigating:

            smach.StateMachine.add('GET_PANEL_CLUSTER',
                            smach_ros.SimpleActionState('get_panel_cluster', PanelPositionAction,
                                result_cb = result_cb), #get_panel_cluster(),
                            transitions={'aborted':'MOVE_CLUSTER_SEARCH','succeeded':'MOVE_PANEL_WAYPOINTS'})

            smach.StateMachine.add('MOVE_CLUSTER_SEARCH', move_cluster_search(),
                            transitions={'succeeded':'GET_PANEL_CLUSTER'})

            smach.StateMachine.add('MOVE_PANEL_WAYPOINTS', move_panel_waypoints(),
                            transitions={'not_found':'failed'})


        self.detect_panel=smach.StateMachine(outcomes=['terminated', 'preempted'])
        with self.detect_panel:
            # Add states to the container
            smach.StateMachine.add('DETECTING_PANEL', detect_panel(),
                            transitions={'found':'MOVE_IN_FRONT_PANEL',
                                         'not_found': 'DETECTING_PANEL',
                                         'preempted': 'preempted'})

            smach.StateMachine.add('MOVE_IN_FRONT_PANEL', move_in_front_panel(),
                            transitions={'succeeded':'terminated'})

        with self.sm_positioning:
            smach.Concurrence.add('CIRCUMNAVIGATING', self.circumnavigating)
            smach.Concurrence.add('DETECTING_PANEL', self.detect_panel)

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('INITIATING', initialization(500000),
                            transitions={'succeeded':'SEARCHING_PANEL'},
                            remapping={'init_params':'user_input',
                                       'init_out':'start_info'})
            # add the concurrent to the main state machine
            smach.StateMachine.add('SEARCHING_PANEL', self.sm_search,
                           transitions={'succeeded':'POSITIONING_IN_FRONT'})

            smach.StateMachine.add('POSITIONING_IN_FRONT', self.sm_positioning,
                           transitions={'succeeded':'DETECTING_VALVE',
                                        'failed':'SEARCHING_PANEL'},
                           #remapping={'position_infront_out':'panel_pose'}
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
    def sm_search_con_termination(self, outcome_map):
        # If the current navigation task has succeeded, return True
        print 'Child_termination'
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
        time.sleep(1)
        return 'succeeded'




    # Define callback function for the concuerrent container
    # Gets called when ANY child state terminates
    def sm_positioning_con_termination(self, outcome_map):
        # If the current navigation task has succeeded, return True
        print 'Positioning termination'
        if outcome_map['CIRCUMNAVIGATING'] == 'failed':
            print 'preempt all the rest'
            return True
        else:
            return False






if __name__ == '__main__':
    Challenge2()
