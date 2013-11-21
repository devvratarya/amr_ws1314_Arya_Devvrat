#!/usr/bin/env python

"""
This module provides a single construct() function which produces a Smach state
machine that implements wallfollowing behavior.

The constructed state machine has three attached methods:
    * set_ranges(ranges): this function should be called to update the range
                          readings
    * get_twist(): returns a twist message that could be directly passed to the
                   velocity publisher
    * set_config(config): updates the machine userdata with the new config

The constructed state machine is preemptable, i.e. each state checks whether
a preemption is requested and returns 'preempted' if that is the case.
"""

PACKAGE = 'amr_bugs'

import roslib
import numpy as np
import matplotlib.pyplot as plt
roslib.load_manifest(PACKAGE)
import smach
from preemptable_state import PreemptableState
from math import copysign
from types import MethodType
from geometry_msgs.msg import Twist


__all__ = ['construct']

#=============================== YOUR CODE HERE ===============================
# Instructions: write a function for each state of wallfollower state machine.
#               The function should have exactly one argument (userdata
#               dictionary), which you should use to access the input ranges
#               and to provide the output velocity.
#               The function should have at least one 'return' statement, which
#               returns one of the possible outcomes of the state.
#               The function should not block (i.e. have infinite loops), but
#               rather it should implement just one iteration (check
#               conditions, compute velocity), because it will be called
#               regularly from the state machine.
#
# Hint: below is an example of a state that moves the robot forward until the
#       front sonar readings are less than the desired clearance. It assumes
#       that the corresponding variables ('front_min' and 'clearance') are
#       available in the userdata dictionary.
#
#           def search(ud):
#               if ud.front_min < ud.clearance:
#                   return 'found_obstacle'
#               ud.velocity = (1, 0, 0)

def find_wall(userdata):
    """Moves the robot forward until it finds a wall."""
    if userdata.front_distance > userdata.clearance:
        userdata.velocity = (0.5, 0, 0)
        return 'front_far'
    else:
        userdata.velocity = (0, 0, 0)
        return 'front_near'


def adjust_to_wall(userdata):
    """Rotates the robot such that it aligns itself with a wall."""
    if userdata.side_distance > userdata.clearance:
        userdata.velocity = (0, 0, -0.5)
        return 'side_sensor_far'
    else:
        userdata.velocity = (0, 0, 0)
        if userdata.front_distance < userdata.clearance:
            return 'side_near_front_near'
        else:
            return 'side_near_front_far'


def go_straight(userdata):
    """Moves the robot straight or returns a message that the robot should turn."""
    if userdata.front_distance < userdata.clearance:
        userdata.velocity = (0, 0, 0)
        return 'front_near'
    elif userdata.side_distance > userdata.clearance:
        if userdata.diag_front > userdata.clearance:
            userdata.velocity = (0.1, 0, 0.5)
            return 'side_far'
        else:
            userdata.velocity = (0.5, 0, 0)
            return 'go_straight'

    userdata.velocity = (0.5, 0, 0)
    return 'go_straight'


def turn_concave(userdata):
    """Turns the robot when it encounters concave obstacles."""
    if userdata.front_distance > (userdata.clearance + 0.1):
        userdata.velocity = (0, 0, 0)
        return 'front_far'
    else:
        userdata.velocity = (0.1, 0, -0.5)
        return 'turn_concave'


def turn_convex(userdata):
    """Turns the robot when it encounters convex obstacles."""
    if userdata.diag_front > userdata.clearance:
        userdata.velocity = (0.1, 0, 0.5)
        return 'turn_convex'
    elif userdata.front_distance > userdata.clearance and userdata.side_distance > userdata.clearance and userdata.back_distance > userdata.clearance and userdata.diag_front > (1.5 *userdata.clearance) and userdata.diag_back > (1.5 * userdata.clearance) and userdata.left_back_distance > userdata.clearance and userdata.right_back_distance > userdata.clearance and userdata.left_front_distance > userdata.clearance and userdata.right_front_distance > userdata.clearance:
        userdata.velocity = (0, 0, 0)
        return 'all_far'
    else:
        userdata.velocity = (0, 0, 0)
        return 'side_near'

#==============================================================================

def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback.
    """
    #============================= YOUR CODE HERE =============================
    # Instructions: store the ranges from a ROS message into the userdata
    #               dictionary of the state machine.
    #               'ranges' is a list or Range messages (that should be
    #               familiar to you by now). It implies that to access the
    #               actual range reading of, say, sonar number 3, you need to
    #               write:
    #
    #                   ranges[3].range
    #
    #               For example, to create an item called 'front_min', which
    #               contains the minimum between the ranges reported by the two
    #               front sonars, you would write the following:
    #
    #                   self.userdata.front_min = min(ranges[3].range, ranges[4].range)
    #
    # Hint: you can just store the whole array of the range readings, but to
    #       simplify the code in your state functions, you may compute
    #       additional values, e.g. the difference between the reading of the
    #       side sonars, or the minimum of all sonar readings, etc.
    #
    # Hint: you can access all the variables stored in userdata. This includes
    #       the current settings of the wallfollower (that is clearance and the
    #       mode of wallfollowing). Think about how you could make your state
    #       functions independent of wallfollowing mode by smart preprocessing
    #       of the sonar readings.

    self.userdata.ranges = ranges
    self.userdata.left_back_distance = ranges[12].range
    self.userdata.right_back_distance = ranges[11].range
    self.userdata.left_front_distance = ranges[3].range
    self.userdata.right_front_distance = ranges[4].range
    if self.userdata.mode == 1:
        self.userdata.back_distance = min(ranges[12].range, ranges[13].range)
        self.userdata.front_distance = min(ranges[4].range, ranges[5].range)
        self.userdata.side_distance = min(ranges[7].range, ranges[8].range)
        self.userdata.diag_front = min(ranges[5].range, ranges[6].range)
        self.userdata.diag_back = min(ranges[9].range, ranges[10].range)
    elif self.userdata.mode == 0:
        self.userdata.back_distance = min(ranges[12].range, ranges[13].range)
        self.userdata.front_distance = min(ranges[2].range, ranges[3].range)
        self.userdata.side_distance = min(ranges[0].range, ranges[15].range)
        self.userdata.diag_front = min(ranges[1].range, ranges[2].range)
        self.userdata.diag_back = min(ranges[13].range, ranges[14].range)

    #==========================================================================


def get_twist(self):
    """
    This function will be attached to the constructed wallfollower machine.
    It creates a Twist message that could be directly published by a velocity
    publisher. The values for the velocity components are fetched from the
    machine userdata.
    """
    twist = Twist()
    twist.linear.x = self.userdata.velocity[0]
    twist.linear.y = self.userdata.velocity[1]
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = self.userdata.velocity[2]
    #============================= YOUR CODE HERE =============================
    # Instructions: although this function is implemented, you may need to
    #               slightly tweak it if you decided to handle wallfolllowing
    #               mode in "the smart way".
    # Hint: state machine userdata is accessible in this function as well, for
    #       example you can read the current wallfollowing mode with
    #
    #           self.userdata.mode
    #

    if self.userdata.mode == 1:
        twist.angular.z = -self.userdata.velocity[2]

    #==========================================================================
    return twist


def set_config(self, config):
    """
    This function will be attached to the constructed wallfollower machine.
    It updates the relevant fields in the machine userdata.
    Its argument is the config object that comes from ROS dynamic reconfigure
    client.
    """
    self.userdata.mode = config['mode']
    self.userdata.clearance = config['clearance']
    return config


def construct():
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.6
    sm.userdata.ranges = None
    # Add states
    with sm:
        #=========================== YOUR CODE HERE ===========================
        # Instructions: construct the state machine by adding the states that
        #               you have implemented.
        #               Below is an example how to add a state:
        #
        #                   smach.StateMachine.add('SEARCH',
        #                                          PreemptableState(search,
        #                                                           input_keys=['front_min', 'clearance'],
        #                                                           output_keys=['velocity'],
        #                                                           outcomes=['found_obstacle']),
        #                                          transitions={'found_obstacle': 'ANOTHER_STATE'})
        #
        #               First argument is the state label, an arbitrary string
        #               (by convention should be uppercase). Second argument is
        #               an object that implements the state. In our case an
        #               instance of the helper class PreemptableState is
        #               created, and the state function in passed. Moreover,
        #               we have to specify which keys in the userdata the
        #               function will need to access for reading (input_keys)
        #               and for writing (output_keys), and the list of possible
        #               outcomes of the state. Finally, the transitions are
        #               specified. Normally you would have one transition per
        #               state outcome.
        #
        # Note: The first state that you add will become the initial state of
        #       the state machine.

        smach.StateMachine.add('FIND_WALL', PreemptableState(find_wall, input_keys=['clearance', 'front_distance'], output_keys=['velocity'], outcomes=['front_far', 'front_near']), transitions= {'front_far': 'FIND_WALL', 'front_near': 'ADJUST_TO_WALL'})

        smach.StateMachine.add('ADJUST_TO_WALL', PreemptableState(adjust_to_wall, input_keys=['clearance', 'front_distance', 'side_distance', 'back_distance', 'diag_front', 'diag_back'], output_keys=['velocity'], outcomes=['side_sensor_far', 'side_near_front_far', 'side_near_front_near', 'back_near_side_far']), transitions= {'side_sensor_far': 'ADJUST_TO_WALL', 'side_near_front_far': 'GO_STRAIGHT', 'side_near_front_near': 'TURN_CONCAVE', 'back_near_side_far': 'TURN_CONVEX'})

        smach.StateMachine.add('GO_STRAIGHT', PreemptableState(go_straight, input_keys=['clearance', 'front_distance', 'side_distance', 'diag_front'], output_keys=['velocity'], outcomes=['front_near', 'side_far', 'all_far', 'go_straight']), transitions= {'go_straight': 'GO_STRAIGHT', 'front_near': 'TURN_CONCAVE', 'side_far': 'TURN_CONVEX', 'all_far': 'FIND_WALL'})

        smach.StateMachine.add('TURN_CONCAVE', PreemptableState(turn_concave, input_keys=['clearance', 'front_distance', 'left_back_distance', 'right_back_distance', 'left_front_distance', 'right_front_distance', 'diag_back'], output_keys=['velocity'], outcomes=['turn_concave', 'front_far', 'far_from_wall']), transitions= {'turn_concave': 'TURN_CONCAVE', 'front_far': 'GO_STRAIGHT', 'far_from_wall': 'ADJUST_TO_WALL'})

        smach.StateMachine.add('TURN_CONVEX', PreemptableState(turn_convex, input_keys=['clearance', 'front_distance', 'back_distance', 'side_distance', 'diag_front', 'diag_back', 'left_back_distance', 'right_back_distance', 'left_front_distance', 'right_front_distance'], output_keys=['velocity'], outcomes=['turn_convex', 'side_near', 'all_far']), transitions= {'turn_convex': 'TURN_CONVEX', 'side_near': 'GO_STRAIGHT', 'all_far': 'FIND_WALL'})

        #======================================================================
    return sm
