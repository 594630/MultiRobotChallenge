#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from bug_2_exercise.srv import GoToPoint, GoToPointResponse
import math

class GoToPointClass:
    def __init__(self):
        #TODO: Initialze a ros node
        rospy.init_node('to_point', anonymous = False)

        #TODO: Create a subscriber to the /odom topic with the callback function self.clbk_odom
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.clbk_odom)

        #TODO: Create a publisher to the /cmd_vel topic assigned to a variable called self.vel_pub
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        #TODO: Create a Service server with the Name "go_to_point_switch" using a custom message structure given in the assignment description and self.go_to_point_switch as the handeling function
        self.service_server = rospy.Service('go_to_point_switch', GoToPoint, self.go_to_point_switch)

        self.position = Point()
        self.yaw = 0
        self.state = 0
        self.desired_position = Point()
        self.yaw_precision = math.pi / 90
        self.dist_precision = 0.05
        self.active = False

    #TODO: Create the handeling function go_to_point_switch which should assign the transmitted boolean value to self.active and the transmitted Point value to self.desired_position
    def go_to_point_switch(self, gtp):
        self.active = gtp.move_switch
        self.desired_position = gtp.target_position
        res = GoToPointResponse()

        return res


    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def change_state(self, state):
        self.state = state
        rospy.loginfo('State changed to '+str(self.state))

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.vel_pub.publish(twist_msg)
        rospy.loginfo('go-to-point -> finished')

    def fix_yaw(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)

        rospy.loginfo(err_yaw)

        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision:
            twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3

        self.vel_pub.publish(twist_msg)
        rospy.loginfo('Yaw error: ['+str(err_yaw)+']')
        rospy.loginfo('Yaw precision: ['+str(self.yaw_precision)+']')
        # state change conditions
        if math.fabs(err_yaw) <= self.yaw_precision:
            rospy.loginfo('Yaw error: ['+str(err_yaw)+']')
            self.change_state(1)

    def go_straight_ahead(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)
        err_pos = math.sqrt(pow(des_pos.y - self.position.y, 2) + pow(des_pos.x - self.position.x, 2))

        if err_pos > self.dist_precision:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            self.vel_pub.publish(twist_msg)
        else:
            rospy.loginfo('Position error: '+str(err_pos))
            self.change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > self.yaw_precision:
            rospy.loginfo('Yaw error: '+str(err_yaw))
            self.change_state(0)

    def main(self):
        rate = rospy.Rate(20)
        first = True

        while not rospy.is_shutdown():
            if not self.active:
                dist_to_target = math.sqrt(pow(self.desired_position.y - self.position.y, 2) + pow(self.desired_position.x - self.position.x, 2))
                if dist_to_target < 0.15:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = 0
                    self.vel_pub.publish(twist_msg)
                rate.sleep()
                continue

            if self.state == 0:
                self.fix_yaw(self.desired_position)
            elif self.state == 1:
                self.go_straight_ahead(self.desired_position)
            elif self.state == 2:
                self.done()
                pass
            else:
                rospy.logerr('Unknown state!')
                pass
            rate.sleep()

if __name__ == '__main__':
    gtp = GoToPointClass()
    gtp.main()
