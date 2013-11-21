#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, ExecutePathAction, \
                         ExecutePathFeedback, ExecutePathResult

from threading import Timer

class MethodPeriodicExecutor(object):
    """Defines a class for executing methods at certain intervals.
    Used from http://stackoverflow.com/questions/3393612/run-certain-code-every-n-seconds
    """
    def __init__(self, interval, function):
        """Initializes a periodic method executor.

        Keyword arguments:
        interval -- Interval at which the function should be executed.
        function -- A callback that should be executed at certain intervals.

        """
        self._timer = None
        self.interval = interval
        self.function = function
        self.is_running = False

    def start(self):
        """Starts the periodic method executor."""
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def _run(self):
        """Runs the periodic method executor."""
        self.is_running = False
        self.start()
        self.function()

    def stop(self):
        """Stops the periodic method executor."""
        self._timer.cancel()
        self.is_running = False

class PathExecutor:
    """Defines a node for handling the behavior of a robot that visits sequences of goal positions."""
    def __init__(self):
        #flags whether the robot should avoid obstacles when it encounters them
        self.uses_obstacle_avoidance = rospy.get_param('~use_obstacle_avoidance')

        self._execute_path_action_server = SimpleActionServer('/path_executor/execute_path', ExecutePathAction, execute_cb=self.execute_cb)
        self._execute_path_action_server.start()

        #clients for handling motion commands
        self.move_to_client_mc = SimpleActionClient('/motion_controller/move_to', MoveToAction)
        self.move_to_client_bug = SimpleActionClient('/bug2/move_to', MoveToAction)

        #a list of booleans that flag whether each goal in a given sequence is reachable or not
        self.nodes_reached = []

        #message returned after the robot has attempted to reach a specific goal
        self.feedback_message = ExecutePathFeedback()

        #message returned after the path execution is completed
        self.result_message = ExecutePathResult()

        #flags whether a certain goal is unreachable or not
        self.unreachable_goal = False

    def execute_cb(self, goal):
        """Callback method used by 'self._execute_path_action_server'.

        Keyword arguments:
        goal -- An 'ExecutePathGoal' object.

        """
        number_of_poses = len(goal.path.poses)
        self.move_to_client_mc.wait_for_server()
        self.move_to_client_bug.wait_for_server()

        publisher = rospy.Publisher('/path_executor/current_path', Path)
        publisher.publish(goal.path)

        #we make an attempt to reach all of the poses in the given sequence,
        #making sure that we react to preemption requests appropriately
        for i in xrange(number_of_poses):

            #we need to make regular checks for preemption while the client is running
            preemption_checker = MethodPeriodicExecutor(1, self._preempt_goal)
            preemption_checker.start()

            message = MoveToGoal()
            message.target_pose = goal.path.poses[i]

            #if we are using obstacle avoidance, then we should use the bug client;
            #otherwise, we are using the motion controller client
            if self.uses_obstacle_avoidance:
                self.move_to_client_bug.send_goal(message, done_cb=self.move_to_done_cb)

                self.move_to_client_bug.wait_for_result()

                self.feedback_message.pose = goal.path.poses[i]
                self._execute_path_action_server.publish_feedback(self.feedback_message)

                #we stop the loop if the robot has reached an unreachable goal and 
                #we don't want to continue the path execution
                if not goal.skip_unreachable and self.unreachable_goal:
                    break
            else:
                self.move_to_client_mc.send_goal(message, done_cb=self.move_to_done_cb)

                self.move_to_client_mc.wait_for_result()

                self.feedback_message.pose = goal.path.poses[i]
                self._execute_path_action_server.publish_feedback(self.feedback_message)

            #we don't check for preemption anymore
            preemption_checker.stop()

        self.result_message.visited = list(self.nodes_reached)
        self._execute_path_action_server.set_succeeded(self.result_message)

    def move_to_done_cb(self, state, result):
        """Callback method used by 'self.move_to_client_mc' and 'self.move_to_client_bug'.
        Note that 'state' equal to 3 denotes that the robot has successfully reached the goal.

        Keyword arguments:
        state -- A number indicating whether the goal was successfully reached or not.
        result -- A 'MoveToResult' object (not used).

        """
        if state == 3:
            self.feedback_message.reached = True
            self.nodes_reached.append(True)
            self.unreachable_goal = False
        else:
            self.feedback_message.reached = False
            self.nodes_reached.append(False)
            self.unreachable_goal = True

    def _preempt_goal(self):
        """Checks whether there is a request for preempting the action server and handles such requests appropriately."""
        if self._execute_path_action_server.is_preempt_requested():
            self.move_to_client_bug.cancel_goal()
            self.move_to_client_mc.cancel_goal()
            self._execute_path_action_server.set_preempted()

if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
