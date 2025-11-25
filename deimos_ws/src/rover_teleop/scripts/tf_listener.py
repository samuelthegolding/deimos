#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf

class PointTransformer:
    def __init__(self):
        rospy.init_node('robot_tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(1.0)  # Transform once every second
        self.transform_point()

    def transform_point(self):
        while not rospy.is_shutdown():
            # Create a point in the base_laser frame that we'd like to transform to the base_link frame
            laser_point = PointStamped()
            laser_point.header.frame_id = "base_laser"
            # We'll just use the most recent transform available
            laser_point.header.stamp = rospy.Time.now()
            # Just an arbitrary point in space
            laser_point.point.x = 0.36
            laser_point.point.y = 0.3
            laser_point.point.z = 0.53

            try:
                # Transform the point from base_laser to base_link
                base_point = self.tf_buffer.transform(laser_point, "base_link", rospy.Duration(1.0))
                rospy.loginfo("base_laser: (%.2f, %.2f, %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                            laser_point.point.x, laser_point.point.y, laser_point.point.z,
                            base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.to_sec)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", str(e))

            self.rate.sleep()

if __name__ == '__main__':
    try:
        transformer = PointTransformer()
    except rospy.ROSInterruptException:
        pass