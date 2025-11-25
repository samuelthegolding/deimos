#!/usr/bin/env python3
 
import rospy
import tf
from nav_msgs.msg import Odometry
 
def odom_callback(msg):
    # Create a transform broadcaster
    br = tf.TransformBroadcaster()
 
    # Position
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
 
    # Broadcast the transform
    br.sendTransform(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
        rospy.Time.now(),
        "base_link",    # child frame
        "odom"          # parent frame
    )
 
 
rospy.init_node('odom_tf_broadcaster')
rospy.Subscriber("/odom", Odometry, odom_callback)
 
rospy.spin()
