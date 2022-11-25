#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math


class WallFollowerClass:
    def __init__(self):
        # TODO: Initialze a ros node
        rospy.init_node('wall_follower', anonymous=False)

        # TODO: Create a subscriber to the /scan topic with the callback function self.clbk_laser
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.clbk_laser)

        # TODO: Create a publisher to the /cmd_vel topic assigned to a variable called self.vel_pub
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # TODO: Create a Service server with the Name "wall_follower_switch" using SetBool as a message structure and self.wall_follower_switch as the handeling function
        rospy.Service('wall_follower_switch', SetBool, self.wall_follower_switch)

        self.active = False

        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state = 0
        self.state_dict = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }

    def clbk_laser(self, msg):
        self.regions = {
            'right': min(min(msg.ranges[180:299]), 1.0),
            'fright': min(min(msg.ranges[300:339]), 1.0),
            'front': min(min(min(msg.ranges[0:19]), min(msg.ranges[340:359])), 1.0),
            'fleft': min(min(msg.ranges[20:59]), 1.0),
            'left': min(min(msg.ranges[60:179]), 1.0),
        }

        self.take_action()

    # TODO: Create the handeling function wall_follower_switch which should assign the transmitted boolean value to self.active
    def wall_follower_switch(self, active):
        self.active = active.data
        res = SetBoolResponse()

        return res

    def change_state(self, state):
        if state is not self.state:
            #    rospy.loginfo('Wall follower - ['+str(state)+'] - '+str(self.state_dict[state]))
            self.state = state

    def take_action(self):
        regions = self.regions

        d = 0.9

        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.3

        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.5

        return msg

    def main(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if not self.active:
                rate.sleep()
                continue
            msg = Twist()
            if self.state == 0:
                msg = self.find_wall()
            elif self.state == 1:
                msg = self.turn_left()
            elif self.state == 2:
                msg = self.follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')

            self.vel_pub.publish(msg)

            rate.sleep()


if __name__ == '__main__':
    wf = WallFollowerClass()
    wf.main()
