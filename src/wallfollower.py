#! /usr/bin/env python

import rospy
import time #?
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from tf import transformations
import tf2_geometry_msgs
from std_msgs.msg import Float64, UInt32

class WallFollower:
	def __init__(self):
		rospy.init_node('wallfollower', anonymous = False)

		rospy.Subscriber("scan", LaserScan, self.clbk_lidar)
		rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.clbk_ar_marker)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

		self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.marker_id_pub = rospy.Publisher('marker_id', UInt32, queue_size=10)

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
		self.marker_id = 100
		while self.marker_id == 100:
			rospy.loginfo("Waitiing for the first marker ID...")
			rospy.sleep(1)

	def clbk_ar_marker(self, msg):
		if len(msg.markers) > 0:
			self.marker_id = msg.markers[0].id
			self.marker_pose = msg.markers[0].pose

	def clbk_lidar(self, msg):
		self.regions = {
			'right': min(min(msg.ranges[180:299]), 1.0),
			'fright': min(min(msg.ranges[300:339]), 1.0),
			'front': min(min(min(msg.ranges[0:19]), min(msg.ranges[340:359])), 1.0),
			'fleft': min(min(msg.ranges[20:59]), 1.0),
			'left': min(min(msg.ranges[60:179]), 1.0),
		}

		self.take_action()

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

	def change_state(self, state):
		if state is not self.state:
			rospy.loginfo('Wall follower - [' + str(state) + '] - ' + str(self.state_dict[state]))
			self.state = state

	def find_wall(self):
		msg = Twist()
		msg.linear.x = 0.2
		msg.angular.z = 0.0
		return msg

	def turn_left(self):
		msg = Twist()
		msg.angular.z = 0.3
		return msg

	def follow_the_wall(self):
		msg = Twist()
		msg.linear.x = 0.5
		return msg

	def run(self):
		rate = rospy.Rate(20)

		while not rospy.is_shutdown():
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
			self.marker_id_pub.publish(self.marker_id)
			rate.sleep()

if __name__ == '__main__':
	wf = WallFollower()
	wf.run()
