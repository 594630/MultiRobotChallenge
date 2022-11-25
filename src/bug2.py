#! /usr/bin/env python

import rospy
import math
import actionlib
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *
from multi_robot_challenge_22.srv import GoToPoint, GoToPointResponse
from multi_robot_challenge_22.msg import bug2_navAction, bug2_navGoal, bug2_navResult, bug2_navFeedback


class bug2():
    def __init__(self):

        rospy.set_param('robot_wait_bigfire', False)

        self.found_wall = False
        # TODO: Initialze a ros node
        rospy.init_node('bug2', anonymous=False)

        # TODO: Create a subscriber to the /scan topic with the callback function self.clbk_laser
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.clbk_laser)

        # TODO: Create a subscriber to the /odom topic with the callback function self.clbk_odom
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.clbk_odom)

        self.srv_client_go_to_point_ = None
        self.srv_client_wall_follower_ = None
        self.yaw_ = 0
        self.yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
        self.dist_precision_ = 0.1

        self.position_ = Point()
        self.desired_position_ = Point()

        self.regions_ = None
        self.state_desc_ = ['Go to point', 'wall following', 'Goal reached']
        # 0 - go to point; 1 - wall following; 2 - Goal reached
        self.state_ = 0

        # TODO: wait for the service servers 'go_to_point_switch' and 'wall_follower_switch' to go active
        rospy.wait_for_service('go_to_point_switch')
        rospy.wait_for_service('wall_follower_switch')

        # TODO: create a service client connected to the 'go_to_point_switch' server
        self.srv_client_go_to_point_ = rospy.ServiceProxy('go_to_point_switch', GoToPoint)

        # TODO: create a service client connected to the 'wall_follower_switch' server
        self.srv_client_wall_follower_ = rospy.ServiceProxy('wall_follower_switch', SetBool)

        # TODO: create an action server with a custom message defined in the assignment description. As an execute_cb use the function self.bug2_execute_cb
        self.aserver = actionlib.SimpleActionServer("bug2_action_server", bug2_navAction,
                                                    execute_cb=self.bug2_execute_cb, auto_start=False)

        # TODO: start the action server
        self.aserver.start()

        self.result = bug2_navResult()
        self.feedback = bug2_navFeedback()

    def clbk_odom(self, msg):
        self.position_ = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    def clbk_laser(self, msg):
        self.regions_ = {
            'right': min(min(msg.ranges[180:299]), 1.0),
            'fright': min(min(msg.ranges[300:339]), 1.0),
            'front': min(min(min(msg.ranges[0:19]), min(msg.ranges[340:359])), 1.0),
            'fleft': min(min(msg.ranges[20:59]), 1.0),
            'left': min(min(msg.ranges[60:179]), 1.0),
        }

    def change_state(self, state):
        self.state_ = state
        log = "state changed: %s" % self.state_desc_[state]
    #    rospy.loginfo(log)
        if self.found_wall == False:
            if self.state_ == 0:
                resp = self.srv_client_go_to_point_(True, self.desired_position_)
                resp = self.srv_client_wall_follower_(False)
            if self.state_ == 1:
                self.last_line_position = self.position_
                resp = self.srv_client_go_to_point_(False, self.desired_position_)
                resp = self.srv_client_wall_follower_(True)
                self.found_wall = True
            if self.state_ == 2:
                resp = self.srv_client_go_to_point_(False, self.desired_position_)
                resp = self.srv_client_wall_follower_(False)
        else:
            resp = self.srv_client_go_to_point_(False, self.desired_position_)
            resp = self.srv_client_wall_follower_(True)
    def distance_to_line(self, p0):
        # p0 is the current position
        # p1 and p2 points define the line
        p1 = self.initial_position_
        p2 = self.desired_position_
        # here goes the equation
        up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
        distance = up_eq / lo_eq

        return distance

    def distance_to_target(self, pos):
        distance = math.sqrt(pow(self.desired_position_.y - pos.y, 2) + pow(self.desired_position_.x - pos.x, 2))
        return distance

    def normalize_angle(self, angle):
        if (math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def bug2_execute_cb(self, msg):
        rospy.loginfo("starting bug2 algorithm")
        # rospy.sleep(3)
        self.initial_position_ = self.position_
        self.desired_position_ = msg.target_position
        # initialize going to the point
        self.change_state(0)#test

        rate = rospy.Rate(20)

        while self.state_ != 2:
            if self.aserver.is_preempt_requested():
                self.aserver.set_preempted()
                rospy.loginfo("Client requested cancelling of the goal")
                break

            if self.regions_ == None:
                continue
            position = self.position_
            distance_position_to_line = self.distance_to_line(position)

            if self.distance_to_target(position) < self.dist_precision_:
                self.change_state(2)
                break

            if self.state_ == 0:
                if self.regions_['front'] > 0.15 and self.regions_['front'] < 1:
                    self.change_state(1)

            elif self.state_ == 1:
                if distance_position_to_line < 0.1:
                    if (self.distance_to_target(position) + 0.15) < self.distance_to_target(self.last_line_position):
                        self.change_state(0)

            #
            self.feedback.current_position = self.position_
            self.aserver.publish_feedback(self.feedback)

            rate.sleep()

        if not self.aserver.is_preempt_requested():
            self.result.base_position = self.position_
            self.aserver.set_succeeded(self.result)

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    b2 = bug2()
    b2.main()
