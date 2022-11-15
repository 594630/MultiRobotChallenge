#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Float64, UInt32
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point
from multi_robot_challenge_22.msg import bug2_navAction, bug2_navGoal, bug2_navResult, bug2_navFeedback


class RobotClass:
    def __init__(self):

        rospy.init_node('robot', anonymous=False)

        # TODO: Create an action client connected to the "bug2_nav_action" action server.
        self.marker_id = None
        self.lidar_value = None
        self.marker_id_pub = None
        self.cmdvel_pub = None
        self.action_client = actionlib.SimpleActionClient("bug2_action_server", bug2_navAction)

        # TODO: Wait for the action server to be active
        self.action_client.wait_for_server()

    def done_cb(self, state, result):
        rospy.loginfo("The robot finished at position: " + str(result.base_position))

    def active_cb(self):
        rospy.loginfo("Bug2 Navigation has started")

    def feedback_cb(self, feedback):
        rospy.loginfo("Current position of the robot: " + str(feedback.current_position))

        rospy.init_node("RobotClass", anonymous=False)
        # test
        rospy.Subscriber("scan", LaserScan, self.clbk_laser)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.clbk_ar_marker)

        self.cmdvel_pub = rospy.Publisher('namespace_com', Float64, queue_size=10)
        self.marker_id_pub = rospy.Publisher('marker_id', UInt32, queue_size=10)

        self.lidar_value = 1000
        self.marker_id = 100

    def clbk_laser(self, msg):
        self.lidar_value = msg.ranges[180]

    def clbk_ar_marker(self, msg):
        if len(msg.markers) > 0:
            self.marker_id = msg.markers[0].id

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # TODO: create an action goal with target_position being a Point with x=0 and y=8
            goal = bug2_navGoal()
            goal.target_position = Point(0, 8, 0)

            # TODO: send the goal to the action server using the existing self.done_cb, self.active_cb and self.feedback_cb functions
            self.action_client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb,
                                         feedback_cb=self.feedback_cb)

            # TODO: wait for the action server to finish
            self.action_client.wait_for_result()

            pass

            self.cmdvel_pub.publish(self.lidar_value)
            self.marker_id_pub.publish(self.marker_id)

            r.sleep()


if __name__ == '__main__':
    robot = RobotClass()
    robot.run()
