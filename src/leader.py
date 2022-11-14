#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64, UInt32
class LeaderClass:
    def __init__(self):
        rospy.init_node("LeaderClass", anonymous=False)

        self.tb3_0_comm_sub = rospy.Subscriber("/tb3_0/marker_id", UInt32, self.clbk_tb3_0_marker_id)
        self.tb3_1_comm_sub = rospy.Subscriber("/tb3_1/marker_id", UInt32, self.clbk_tb3_1_marker_id)

        self.tb3_0_marker_id = 100
        self.tb3_1_marker_id = 100

        self.tags = [False, False, False, False, False]

    def clbk_tb3_0_marker_id(self, msg):
        self.tb3_0_marker_id = msg.data

    def clbk_tb3_1_marker_id(self, msg):
        self.tb3_1_marker_id = msg.data

    def run(self):
        r = rospy.Rate(1)
        rospy.set_param('/start_flag', True)

        while not rospy.is_shutdown():
            rospy.loginfo("TB3_0 Marker ID: %s", self.tb3_0_marker_id)
            rospy.loginfo("TB3_1 Marker ID: %s", self.tb3_1_marker_id)

            if self.tb3_0_marker_id < 5 and not self.tags[self.tb3_0_marker_id]:
                self.tags[self.tb3_0_marker_id] = True
                if self.tb3_0_marker_id == 2:
                    rospy.set_param('ar_human', [{'id': self.tb3_0_marker_id, 'position': {'x': 1, 'y': 2, 'z': 3}}])
                elif self.tb3_0_marker_id == 4:
                    rospy.set_param('ar_big_fire', [{'id': self.tb3_0_marker_id, 'position': {'x': 1, 'y': 2, 'z': 3}}])
                else:
                    rospy.set_param('ar_fire', [{'id': self.tb3_0_marker_id, 'position': {'x': 1, 'y': 2, 'z': 3}}])

            if self.tb3_1_marker_id < 5 and not self.tags[self.tb3_1_marker_id]:
                self.tags[self.tb3_1_marker_id] = True
                if self.tb3_1_marker_id == 2:
                    rospy.set_param('ar_human', [{'id': self.tb3_1_marker_id, 'position': {'x': 1, 'y': 2, 'z': 3}}])
                elif self.tb3_1_marker_id == 4:
                    rospy.set_param('ar_big_fire', [{'id': self.tb3_1_marker_id, 'position': {'x': 1, 'y': 2, 'z': 3}}])
                else:
                    rospy.set_param('ar_fire', [{'id': self.tb3_1_marker_id, 'position': {'x': 1, 'y': 2, 'z': 3}}])

            r.sleep()

if __name__ == '__main__':
    robot = LeaderClass()
    robot.run()
