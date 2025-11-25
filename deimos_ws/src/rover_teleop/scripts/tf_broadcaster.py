#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations

class TransformBroadcaster:
    def __init__(self):
        rospy.init_node('robot_tf_publisher')
        self.br = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(100.0)

    def publish_transform(self):
        while not rospy.is_shutdown():
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = "base_laser"
            t.transform.translation.x = 0.1
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.2
            # Quaternion representing no rotation (0, 0, 0, 1)
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.br.sendTransform(t)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        transform_broadcaster = TransformBroadcaster()
        transform_broadcaster.publish_transform()
    except rospy.ROSInterruptException:
        pass