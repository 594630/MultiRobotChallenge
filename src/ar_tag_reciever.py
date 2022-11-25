#! /usr/bin/env python

import rospy
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import tf2_geometry_msgs

class ARTagClass:
    def __init__(self):
        # Initiating ROS Node
        rospy.init_node('ar_tag_reciever', anonymous=False )

        # Setting up a listener to lookup transformations between different coordinate frames
        self.tfBuffer = tf2_ros.Buffer() #self.tfBuffer
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscriber to the topic that will report when an AR Tag was recognized in the camera output.
        # The topic name might change depending on in which Namespace this script is launched.
        # If it is launched outside of the robots namespace it could be e.g. '/tb3'
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.clbk_ar_marker)

        # The marker_list will be used to check that each marker_id was found and that the same marker_id is not reported multiple times.
        self.marker_list = [False, False, False, False, False]

        # Setting a default value for self.marker_id and waiting until it's updated by the ar_pose_marker Subscriber
        rospy.set_param('/start_flag', True)
        rospy.loginfo("start flag set")
        self.marker_id = 100
        while self.marker_id == 100:
            rospy.loginfo("Waitiing for the first marker ID...")
            rospy.sleep(1)
        # Setting a default value for self.marker_id and waiting until it's updated by the ar_pose_marker Subscriber


    # Callback function for the Subscriber of the 'ar_pose_marker' topic
    def clbk_ar_marker(self, msg):
        if len(msg.markers) > 0:
            self.marker_id = msg.markers[0].id
            self.marker_pose = msg.markers[0].pose



    def run(self):
        #Setting the rate at which the while loop will run to 1 Hz
        r = rospy.Rate(1)

        #Starting the scoring
        rospy.set_param('/start_flag', True)

        while not rospy.is_shutdown():
            # Marker ID
            rospy.loginfo("Marker ID: %s", self.marker_id)
            # Position of the Marker in the Camera Frame
            rospy.loginfo("Marker Pos: %s", self.marker_pose.pose.position)

            # Saving the current marker_id and marker_pose in local variables so that they cannot change during the execution of this loop iteration.
            marker_id = self.marker_id
            marker_pose = self.marker_pose

            # Variable that is used to stop the while loop
            finished = False

            # Making sure that the marker_id is a value from 0 to 4
            if marker_id < 0 or marker_id > 4:
                r.sleep()
                continue

            # If a marker_id is detected for the first time
            if not self.marker_list[marker_id]:
                # Trying to lookup the current transformation from the camera frame to the world frame
                try:
                    self.trans_camera_odom = self.tfBuffer.lookup_transform("map", "tb3_0/camera_rgb_optical_frame", rospy.Time())
                    # break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    r.sleep()
                    continue

                # Transforming the position and orientation of the marker (marker_pose) in the camera frame to the pose in the world frame.
                marker_odom_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, self.trans_camera_odom)
                # Extracting only the position part of the pose
                marker_odom_position = marker_odom_pose.pose.position
                rospy.loginfo("Transformed Pos: %s", marker_odom_pose.pose.position)

                # If one of the fire id's is deteceted it is reported to the scoring script
                if marker_id == 0 or marker_id == 1 or marker_id == 3 or marker_id == 4:
                    rospy.set_param('/ar_fire', {'id': marker_id, 'position': {'x': marker_odom_position.x, 'y': marker_odom_position.y, 'z': marker_odom_position.z}})
                # If the human id is detected it is reported to the scoring script
                elif marker_id == 2:
                    rospy.set_param('/ar_human', {'id': marker_id, 'position': {'x': marker_odom_position.x, 'y': marker_odom_position.y, 'z': marker_odom_position.z}})

                # Updating the marker_list to ensure that each AR tag is only reported once.
                self.marker_list[marker_id] = True

            # Checking if all IDs have been found
            finished = True
            for m in self.marker_list:
                if not m:
                    finished = False

            # If all AR Tags have been found exit the while loop
            if finished:
                rospy.set_param('/finish_flag', True)
                break

            # Sleep after each while loop iteration to ensure a frequency of 1 Hz
            r.sleep()

if __name__ == '__main__':
    artag = ARTagClass()
    artag.run()
