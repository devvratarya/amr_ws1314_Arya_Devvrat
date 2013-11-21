#!/usr/bin/env python

from planar import Point, Vec2
from planar.c import Line
from math import degrees
import math

#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty BugBrain class. A new instance of
#               this class will be created for each new move_to command. The
#               constructor receives the goal specifiaction and the mode of
#               wallfollowing (left (0) or right (1)) that is currently in use.
#               All the remaining functions receive the current position and
#               orientation of the robot.
#d
# Hint: you can create a class member variable at any place in your code (not
#       only in __init__) by assigning a value to it, e.g.:
#
#           self.some_member_variable = 2012
#
# Hint: you could use the 'planar' library to avoid implementing geometrical
#       functions that check the distance between a point and a line, or any
#       other helper functions that you need. To use its classes add the
#       following import statements on top of the file:
#
#            from planar import Point, Vec2
#            from planar.c import Line
#            from math import degrees
#
#       As discussed in the lab class, you will need to install the library by
#       executing `sudo pip install planar` in the terminal.
#
# Hint: all the member variables whose names start with 'wp_' (which stands for
#       'waypoint') will be automagically visualized in RViz as points of
#       different colors. Similarly, all the member variables whose names
#       start with 'ln_' (which stands for 'line') will be visualized as lines
#       in RViz. The only restriction is that the objects stored in these
#       variables should indeed be points and lines.
#       The valid points are:
#
#           self.wp_one = (1, 2)
#           self.wp_two = [1, 2]
#           self.wp_three = Point(x, y) # if you are using 'planar'
#
#       The valid lines are (assuming that p1 and p2 are valid points):
#
#           self.ln_one = (p1, p2)
#           self.ln_two = [p1, p2]
#           self.ln_three = Line.from_points([p1, p2]) # if you are using 'planar'

class BugBrain:

    TOLERANCE = 0.025
    wp_leave_wall_array = []
    leave_wall_check = 0
    follow_wall_check = 0
    wp_follow_wall_array = []
    is_left_line = 0

    def __init__(self, goal_x, goal_y, side):
		self.current_x = 0
		self.current_y = 0
		self.goal_x = goal_x
		self.goal_y = goal_y
        #pass

    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        print "follow wall"
        self.follow_x = x
        self.follow_y = y
        BugBrain.follow_wall_check = 1
        self.wp_follow = Point(self.follow_x,self.follow_y)
        BugBrain.leave_wall_check = 0
        BugBrain.wp_follow_wall_array.append(self.wp_follow)
        self.wp_goal = Point(self.goal_x, self.goal_y)
        self.ln_goal = Line.from_points([self.wp_follow,self.wp_goal])
        # compute and store necessary variables
        #pass

    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        #print "leaving wall"
        #self.current_x = x
        #self.current_y = y
        #self.wp_current_leave = Point(self.current_x,self.current_y)
        #BugBrain.wp_leave_wall_array.append(self.wp_current_leave)
        #print len(BugBrain.wp_leave_wall_array)	
        # compute and store necessary variables
        #pass

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        self.current_x = x
        self.current_y = y
        self.wp_goal_unreachable = Point(self.current_x,self.current_y)
        self.dist_btw_follow_goal_unreachable = abs(self.wp_goal_unreachable.distance_to(self.wp_follow))
        #print self.is_left_line
        #print self.dist_btw_follow_goal_unreachable
        if self.dist_btw_follow_goal_unreachable < self.TOLERANCE and self.is_left_line == 1:
            print "goal unreachable"
            return True
        else:
            return False

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """
        self.current_x = x
        self.current_y = y
        self.is_left_line = 0
        self.wp_current_leave = Point(self.current_x,self.current_y)
        self.current_dist_to_line = abs(self.ln_goal.distance_to(self.wp_current_leave))
        if self.current_dist_to_line > (self.TOLERANCE + 0.5):
            self.is_left_line = 1
        self.dist_btw_follow_current = self.wp_current_leave.distance_to(self.wp_follow)
        if self.wp_current_leave == self.wp_follow:
            return False
        elif self.dist_btw_follow_current < self.TOLERANCE:
            return False
        elif self.follow_wall_check == 1 and self.current_dist_to_line < self.TOLERANCE and self.dist_btw_follow_current > self.TOLERANCE:		
            #print len(BugBrain.wp_leave_wall_array)
            #BugBrain.wp_leave_wall_array.append(self.wp_current_leave)
            BugBrain.follow_wall_yn = 0
            BugBrain.leave_wall_check = 1
            self.is_left_line = 0
            print BugBrain.leave_wall_check
            return True
        else:
            return False
