#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist:
    def __init__(self):
        rospy.init_node('joy_to_twist', anonymous=True)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("j0", Joy, self.joy_callback)

        # Axis mappings (adjust these based on your F310 controller - check 'rostopic echo /joy')
        self.axis_linear = 5  # Axis for forward/backward (usually vertical)
        self.axis_angular = 4 # Axis for turning (usually horizontal)

        # Speed scaling factors (adjust these to control sensitivity)
        self.linear_speed_scale = 0.2  # m/s per unit of joystick input
        self.angular_speed_scale = 0.5 # rad/s per unit of joystick input

        # Store last twist message to prevent sending stale data
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0

    def joy_callback(self, joy_data):
        # Get linear velocity from the specified axis
        linear_speed = joy_data.axes[self.axis_linear] * self.linear_speed_scale

        # Get angular velocity from the specified axis
        angular_speed = joy_data.axes[self.axis_angular] * self.angular_speed_scale

        # Update the twist message only if the joystick values change
        self.last_linear_speed = linear_speed
        self.last_angular_speed = angular_speed

    def run(self):
        rate = rospy.Rate(10)  # Publish at 10 Hz
        while not rospy.is_shutdown():
            # Create and publish a new Twist message with the current velocities
            twist_msg = Twist()
            twist_msg.linear.x = self.last_linear_speed
            twist_msg.angular.z = self.last_angular_speed
            self.twist_pub.publish(twist_msg)
            rate.sleep()

if __name__ == '__main__':
    joy_to_twist = JoyToTwist()
    joy_to_twist.run()
