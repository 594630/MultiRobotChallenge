#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from tf import transformations
from nav_msgs.msg import Odometry
#Earlier solution
class GoToPoint:
    def __init__(self):
        rospy.init_node('topointnode', anonymous = False)

        self.odomsub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.velpub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.robot_pos = 123
        self.robot_orientation = 123
        self.robot_x = 123
        self.robot_y = 123

    def odom_callback(self, msg):
        self.robot_pos = msg.pose.pose.position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)

        self.robot_orientation = euler[2]

    def run(self):
        vel_msg = Twist()
        r = rospy.Rate(10)
        point = Point()
        point.x = 4
        point.y = -4
        distance = sqrt(pow(point.x - self.robot_x, 2) + pow(point.y - self.robot_y, 2))

        inc_x = point.x - self.robot_x
        inc_y = point.y - self.robot_y
        angle_to_point = atan2(inc_y, inc_x)

        while abs(angle_to_point - self.robot_orientation) > 0.1:
            dir = (angle_to_point - self.robot_orientation) / abs(angle_to_point - self.robot_orientation)
            angle_speed = min(0.5, abs(angle_to_point - self.robot_orientation) / 1.5)
            vel_msg.angular.z = dir * angle_speed
            vel_msg.linear.x = 0
            self.velpub.publish(vel_msg)
            r.sleep()

        vel_msg.angular.z = 0
        while distance >= 0.7:
            distance = sqrt(pow(point.x - self.robot_x, 2) + pow(point.y - self.robot_y, 2))
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = 0
            self.velpub.publish(vel_msg)
            r.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velpub.publish(vel_msg)

if __name__ == '__main__':
    gtp = GoToPoint()
    gtp.run()
