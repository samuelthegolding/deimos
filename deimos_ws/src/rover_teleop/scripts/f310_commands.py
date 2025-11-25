#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# Create the ROS publisher for motor PWM control (left and right motors in a single message)
motor_pwm_pub = rospy.Publisher('/motor_pwm', Float32MultiArray, queue_size=10)

def joy_callback(data):
    # Get the vertical axis (forward/backward movement)
    forward_backward = data.axes[1]  # Axis 1 for forward/backward (up-down joystick)
    
    # Get the horizontal axis (turning left/right)
    left_right = data.axes[0]  # Axis 0 for turning (left-right joystick)

    # Calculate the left and right PWM values based on joystick inputs
    # Both motors should move in the same direction for forward/backward
    pwm_left = -forward_backward * 100 + 1500
    pwm_right = forward_backward * 100 + 1500

    # Now handle turning:
    # For turning, one motor should go forward and the other backward (mirror behavior)
    if left_right < 0:  # Turning right
        pwm_left -= left_right * 100
        pwm_right += -left_right * 100
    elif left_right > 0:  # Turning left
        pwm_left += -abs(left_right) * 100
        pwm_right -= abs(left_right) * 100

    # Ensure PWM values stay within valid range (1000-2000)
    pwm_left = max(1000, min(2000, pwm_left))
    pwm_right = max(1000, min(2000, pwm_right))

    # Create a Float32MultiArray message to hold both PWM values
    pwm_message = Float32MultiArray()
    pwm_message.data = [pwm_left, pwm_right]  # Left and right PWM values in a list

    # Publish the motor PWM values in the single topic
    motor_pwm_pub.publish(pwm_message)

def listener():
    # Initialize ROS node
    rospy.init_node('joy_to_motor_pwm', anonymous=True)
    
    # Subscribe to the joystick input topic
    rospy.Subscriber("joy", Joy, joy_callback)
    
    # Keep the node running to process incoming joystick data
    rospy.spin()

if __name__ == '__main__':
    listener()

