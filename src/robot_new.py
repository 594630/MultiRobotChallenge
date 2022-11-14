#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64, UInt32
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers

class RobotClass:
    def __init__(self):
        rospy.init_node("RobotClass", anonymous=False)

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

            self.cmdvel_pub.publish(self.lidar_value)
            self.marker_id_pub.publish(self.marker_id)

            r.sleep()

if __name__ == '__main__':
    robot = RobotClass()
    robot.run()
