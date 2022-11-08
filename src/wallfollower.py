#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
PI = 3.1415926535897
proxi_limit = 0.65
#Earlier solution
class WallFollower:
	def __init__(self):
		rospy.init_node('wallfollower', anonymous = False)

		self.lidarsub = rospy.Subscriber("/scan", LaserScan, self.clbk_lidar)
		self.velpub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

		#default values
		self.lidar_left = 100
		self.lidar_front = 100
		self.lidar_right = 100
		self.lidar_behind = 100
		self.lidar_45deg = 100
		self.lidar_315deg = 100

	def clbk_lidar(self, msg):
		self.lidar_left = msg.ranges[90]
		self.lidar_front = msg.ranges[0]
		self.lidar_right = msg.ranges[270]
		self.lidar_behind = msg.ranges[180]
		self.lidar_45deg = msg.ranges[44]
		self.lidar_315deg = msg.ranges[314]

	def runNavigation(self):
		vel_msg = Twist()
		r = rospy.Rate(10)
		vel_msg.linear.x = 0.3
		vel_msg.angular.z = 0
		finished = False

		while self.lidar_front > proxi_limit and self.lidar_left > proxi_limit and self.lidar_right > proxi_limit and self.lidar_behind > proxi_limit:
			vel_msg.linear.x = 0.3
			vel_msg.angular.z = 0
			self.velpub.publish(vel_msg)

		while not finished:

			def rotate(self):
				degrees_per_sec = 42
				target_angle = 90
				angular_degrees = degrees_per_sec*2*PI/360
				relative_angle = target_angle*2*PI/360
				vel_msg.linear.x = 0
				vel_msg.angular.z = -abs(angular_degrees)

				start_time = rospy.Time.now().to_sec()
				current_angle = 0

				while(current_angle < relative_angle):
					self.velpub.publish(vel_msg)
					current_time = rospy.Time.now().to_sec()
					current_angle = angular_degrees*(current_time-start_time)

				vel_msg.angular.z = 0
				self.velpub.publish(vel_msg)

			if self.lidar_front < proxi_limit and self.lidar_45deg > self.lidar_front and self.lidar_315deg > self.lidar_front:
				rotate(self)
				vel_msg.linear.x = 0.35

			if self.lidar_45deg > proxi_limit:
				vel_msg.linear.x = 0.3
				vel_msg.angular.z = 0.6
			elif self.lidar_45deg < proxi_limit:
				vel_msg.linear.x = 0.3
				vel_msg.angular.z = -0.6

			self.velpub.publish(vel_msg)
			r.sleep()

if __name__ == '__main__':
	wf = WallFollower()
	wf.runNavigation()
