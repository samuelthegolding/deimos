#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, ChannelFloat32, PointField
from sensor_msgs import point_cloud2
import std_msgs.msg
import geometry_msgs.msg
import numpy as np
import struct

class PointCloudConverter:
    def __init__(self):
        rospy.init_node('realsense_to_pointcloud_py', anonymous=True)
        self.cloud_pub = rospy.Publisher('cloud', PointCloud, queue_size=1)
        self.cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)

    def cloud_callback(self, cloud_msg2):
        rospy.loginfo_once(f"Received PointCloud2 message with fields: {cloud_msg2.fields}")
        cloud_msg = PointCloud()
        cloud_msg.header = cloud_msg2.header
        cloud_msg.points = []
        cloud_msg.channels = []

        try:
            points = point_cloud2.read_points(cloud_msg2, field_names=("x", "y", "z", "rgb"), skip_nans=True)

            intensities = []
            for p in points:
                point = geometry_msgs.msg.Point32()
                point.x = p[0]
                point.y = p[1]
                point.z = p[2]
                cloud_msg.points.append(point)
                rgb_float = p[3]
                s = struct.pack('>f', rgb_float)
                i = struct.unpack('>i', s)[0]
                r = (i >> 16) & 0xFF
                g = (i >> 8) & 0xFF
                b = i & 0xFF
                intensity = (r + g + b) / 3.0
                intensities.append(intensity)

            if cloud_msg.points and intensities:
                intensity_channel = ChannelFloat32()
                intensity_channel.name = "intensities"
                intensity_channel.values = intensities
                cloud_msg.channels.append(intensity_channel)

            self.cloud_pub.publish(cloud_msg)

        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")

if __name__ == '__main__':
    converter = PointCloudConverter()
    rospy.spin()
