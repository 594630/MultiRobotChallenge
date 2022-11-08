#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point
from bug_2_exercise.msg import bug2_navAction, bug2_navGoal, bug2_navResult, bug2_navFeedback

class RobotClass():
    def __init__(self):
        #TODO: Initialze a ros node
        rospy.init_node('robot', anonymous = False)

        #TODO: Create an action client connected to the "bug2_nav_action" action server.
        self.action_client = actionlib.SimpleActionClient("bug2_action_server", bug2_navAction)

        #TODO: Wait for the action server to be active
        self.action_client.wait_for_server()

        pass


    def done_cb(self, state, result):
        rospy.loginfo("The robot finished at position: "+str(result.base_position))

    def active_cb(self):
        rospy.loginfo("Bug2 Navigation has started")

    def feedback_cb(self, feedback):
        rospy.loginfo("Current position of the robot: "+str(feedback.current_position))


    def run(self):
        #TODO: create an action goal with target_position being a Point with x=0 and y=8
        goal = bug2_navGoal()
        goal.target_position = Point(0, 8, 0)

        #TODO: send the goal to the action server using the existing self.done_cb, self.active_cb and self.feedback_cb functions
        self.action_client.send_goal(goal, done_cb = self.done_cb, active_cb = self.active_cb, feedback_cb = self.feedback_cb)

        #TODO: wait for the action server to finish
        self.action_client.wait_for_result()

        pass


if __name__=="__main__":
    robot = RobotClass()
    robot.run()
