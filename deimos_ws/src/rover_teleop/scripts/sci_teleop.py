#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Int32

# Publishers
science_module_pub = rospy.Publisher('/science_module', Float32MultiArray, queue_size=10)
carousel_pub = rospy.Publisher('/carousel_count', Int32, queue_size=10)
syringe_pub = rospy.Publisher('/syringe_count', Int32, queue_size=10)

# Conversion factor: 6400 steps/rev → 1° ≈ 17.78 counts
COUNTS_PER_DEGREE = 6400 / 360.0

def joy_callback(data):
    # === Auger and Actuator PWM ===
    auger_speed = data.axes[2]
    actuator_position = data.axes[1]
    auger_pwm = max(1000, min(2000, auger_speed * 200 + 1500))
    actuator_pwm = max(1000, min(2000, actuator_position * 300 + 1500))

    pwm_msg = Float32MultiArray()
    pwm_msg.data = [auger_pwm, actuator_pwm]
    science_module_pub.publish(pwm_msg)

    # === Syringe Stepper Control (buttons 0 and 2) ===
    if data.buttons[1]:  # A button → +10°
        syringe_pub.publish(Int32(data=int(COUNTS_PER_DEGREE * 1440)))
    if data.buttons[3]:  # X button → -10°
        syringe_pub.publish(Int32(data=int(-COUNTS_PER_DEGREE * 1440)))

    # === Carousel Stepper Control (buttons 1 and 3) ===
    if data.buttons[0]:  # B button → +60°
        carousel_pub.publish(Int32(data=int(COUNTS_PER_DEGREE * 60)))
    if data.buttons[2]:  # Y button → -60°
        carousel_pub.publish(Int32(data=int(-COUNTS_PER_DEGREE * 60)))

def listener():
    rospy.init_node('science_module_control', anonymous=True)
    rospy.Subscriber("j1", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
