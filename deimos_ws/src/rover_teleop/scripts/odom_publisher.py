#!/usr/bin/env python3
 
from __future__ import print_function
 
import threading
 
import roslib
import rospy
import tf
import numpy as np 
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
 
#wheel_positions = None  # Global array for the [Left, Right] wheel positions
wheel_angvel = None
 
def wheel_cb(msg: Float32MultiArray):
    global wheel_angvel
    wheel_angvel = msg.data
 
 
 
if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    sub = rospy.Subscriber("/wheel_positions", Float32MultiArray, callback=wheel_cb)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(50)   # 10 Hz
 
    x = 0.0
    y = 0.0
    th = 0.0
 
    vx = 0.1
    vy = 0.0
    vth = 0.1
 
    current_time = rospy.Time.now()
    last_time = rospy.Time.now() 
    current_wheelangvel = wheel_angvel
    last_wheelangvel = wheel_angvel
    wheel_diameter = 0.3  # diam in meters
    pulsePerRevolution = 8192
 
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        current_wheelangvel = wheel_angvel
 
        # compute odometry here (for example purposes we increment)
        dt = (current_time - last_time).to_sec()
        if current_wheelangvel is not None and last_wheelangvel is not None:
            leftWheelAngularVel = -current_wheelangvel[0]
            rightWheelAngularVel =-current_wheelangvel[1]
            leftWheelVel = rightWheelAngularVel*wheel_diameter/2
            rightWheelVel = leftWheelAngularVel*wheel_diameter/2
 
            vx = (leftWheelVel + rightWheelVel)/2 # average velocity
            vy = 0  
            vth = (leftWheelVel - rightWheelVel)/.483
            delta_x = vx * dt
            delta_y = vy * dt
            delta_th = vth * dt
 
            x += delta_x
            y += delta_y
            th += delta_th
        else:
            pass
        # since quaternion is needed
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th+math.pi)
 
        # publish the transform
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )
 
        # publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
 
        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
 
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
 
        # publish
        odom_pub.publish(odom)
 
        last_time = current_time
        last_wheelangvel = current_wheelangvel
        rate.sleep()
