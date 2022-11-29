#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64, UInt32


class LeaderClass:
    def __init__(self):
        rospy.init_node("LeaderClass", anonymous=False)

        self.tb3_0_comm_sub = rospy.Subscriber("/tb3_0/namespace_com", Float64, self.clbk_tb3_0_comm)
        self.tb3_1_comm_sub = rospy.Subscriber("/tb3_1/namespace_com", Float64, self.clbk_tb3_1_comm)
        self.tb3_1_comm_sub = rospy.Subscriber("/tb3_2/namespace_com", Float64, self.clbk_tb3_2_comm)

        self.tb3_0_comm_sub = rospy.Subscriber("/tb3_0/marker_id", UInt32, self.clbk_tb3_0_marker_id)
        self.tb3_1_comm_sub = rospy.Subscriber("/tb3_1/marker_id", UInt32, self.clbk_tb3_1_marker_id)
        self.tb3_1_comm_sub = rospy.Subscriber("/tb3_2/marker_id", UInt32, self.clbk_tb3_2_marker_id)

        self.tb3_0_lidar_value = 1000
        self.tb3_1_lidar_value = 1000
        self.tb3_2_lidar_value = 1000

        self.tb3_0_marker_id = 100
        self.tb3_1_marker_id = 100
        self.tb3_2_marker_id = 100

    def clbk_tb3_0_comm(self, msg):
        self.tb3_0_lidar_value = msg.data

    def clbk_tb3_1_comm(self, msg):
        self.tb3_1_lidar_value = msg.data

    def clbk_tb3_2_comm(self, msg):
        self.tb3_2_lidar_value = msg.data

    def clbk_tb3_0_marker_id(self, msg):
        self.tb3_0_marker_id = msg.data

    def clbk_tb3_1_marker_id(self, msg):
        self.tb3_1_marker_id = msg.data

    def clbk_tb3_2_marker_id(self, msg):
        self.tb3_2_marker_id = msg.data

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            #     rospy.loginfo("TB3_0 Lidar Value: %s", self.tb3_0_lidar_value)
            #    rospy.loginfo("TB3_1 Lidar Value: %s", self.tb3_1_lidar_value)
            #   rospy.loginfo("TB3_0 Marker ID: %s", self.tb3_0_marker_id)
            #  rospy.loginfo("TB3_1 Marker ID: %s", self.tb3_1_marker_id)

            r.sleep()


if __name__ == '__main__':
    robot = LeaderClass()
    robot.run()
