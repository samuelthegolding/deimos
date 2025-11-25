#!/usr/bin/env python3

from __future__ import print_function

import threading


import rospy

from std_msgs.msg import UInt16MultiArray, String
import sys, select, termios, tty
from sensor_msgs.msg import Joy

import math
import numpy as np
msg = """
Modified version of teleop_twist_keyboard for joystick by 
https://github.com/ros-teleop/teleop_twist_keyboard

Reading from the joystick and Publishing to /servo_positions!
---------------------------
Moving around:
Y on Rstick : Forward/back
X on Rstick : Rotate arm base
Y on Lstick : Up/down
X on Lstick : Wrist rotate
Y on Crosspad : Hand Up/down

RT : Open hand
LT : Close hand

CTRL-C to quit
"""

dofBounds = [
    [5000, 8000], # dof 0
    [4400, 8000], # dof 1
    [4200, 7000], # dof 2
    [6000, 7900], # dof 3
    [4000, 8000], # dof 4
    [3000, 9000]  # dof 5
]

r_pos = 12.8  # r, z, theta values measured from dof0 to dof3 joint
z_pos = 10.0
#r_pos = 10.72  # r, z, theta values measured from dof0 to dof3 joint
#z_pos = 10.35
theta_pos = 0.0

Hand_Count = 0
joyJoystick =[0]*6
joyButtons = [0]*12

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/servo_positions', UInt16MultiArray, queue_size = 1)
        self.dof0 = 6000 # Neutral staring position for DOF
        self.dof1 = 7900 
        self.dof2 = 4300
        self.dof3 = 6000
        self.dof4 = 4000
        self.dof5 = 6000
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, dof0, dof1, dof2, dof3, dof4, dof5):
        self.condition.acquire()
        self.dof0 = dof0
        self.dof1 = dof1
        self.dof2 = dof2
        self.dof3 = dof3
        self.dof4 = dof4
        self.dof5 = dof5
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        #self.done = True
        self.update(6000, 7900, 4300, 6000, 4000, 6000)
        #self.join()

    def run(self):
        servo_msg = UInt16MultiArray()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            servo_msg.data = [self.dof0, self.dof1, self.dof2, self.dof3, self.dof4, self.dof5]

            self.condition.release()

            # Publish.
            self.publisher.publish(servo_msg)

        # Publish stop message when thread exits.
        servo_msg.data = [self.dof0, self.dof1, self.dof2, self.dof3, self.dof4, self.dof5]
        self.publisher.publish(servo_msg)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def servo_step(current_position, step_size, position_bounds):
    updated_position = current_position + step_size
    if(updated_position < position_bounds[0] or updated_position > position_bounds[1]):
        return current_position  # dont update if outside max/min positions
    return updated_position

def joy_callback(data):
    global joyJoystick, joyButtons
    joyJoystick = data.axes # Store positions of joysticks
    joyButtons = data.buttons

def joy_listener():
    rospy.Subscriber('/j1', Joy, joy_callback)

def done_cb(data):
    rospy.loginfo("Received from Arduino: %s", data.data)

def key_listener():
    while not rospy.is_shutdown():
        key = getKey(key_timeout)  
        if key == 'q':
            pub_thread.stop()
            break

def update_coordinates(r, z, theta, axes, buttons):
    r_pos = r + axes[3]*(1-0.5*buttons[5])*0.1
    z_pos = z + axes[1]*(1-0.5*buttons[5])*0.1
    theta_pos = theta - axes[2]*(1-0.9*buttons[5])*0.25
    l3 = math.sqrt((r_pos-6.16)**2+(z_pos-6.38)**2) 

    if(l3>33.7 or l3<1.6):
        return r, z, theta_pos
    return r_pos, z_pos, theta_pos

def calculate_positions(r,z, theta):
    l1 = 17.63
    l2 = 16.12

    r_offset = r - 6.16  # distance from DOF0 to DOF1
    z_offset = z - 6.38  # distance from DOF0 to DOF1

    l3 = math.sqrt(r_offset**2 + z_offset**2)

    def safe_acos(x):
        return math.acos(min(1.0, max(-1.0, x)))

    g1 = math.degrees(safe_acos((l1**2 + l3**2 - l2**2) / (2 * l1 * l3)))
    g2 = math.degrees(safe_acos((l1**2 + l2**2 - l3**2) / (2 * l1 * l2)))

    phi1 = math.degrees(math.atan2(z_offset, r_offset))  # Safer than atan
    theta1 = np.clip(g1 + phi1, -7, 105)
    g2 = np.clip(g2, 20, 140)

    dof0 = np.uint16(theta * 11.94 + 6000)
    dof1 = np.uint16(32.143 * theta1 + 4625)
    dof2 = np.uint16(23.3 * g2 + 3733)

    return dof0, dof1, dof2

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('arm_teleop_joy')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)
    joy_listener()
    #key_thread = threading.Thread(target=key_listener, daemon=True)
    #key_thread.start()
    dof0 = 6050
    dof1 = 7900
    dof2 = 4300
    dof3 = 6000
    dof4 = 4000
    dof5 = 6000

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(dof0, dof1, dof2, dof3, dof4, dof5)
        print(msg)
        Hand_Count = 0
        while not rospy.is_shutdown():
            
            if(Hand_Count>=(500000/2)): # Change position every 500000 cycles since
                                    # the loop runs too frequently
                Hand_Count = 0  # Reset count
                r_pos, z_pos, theta_pos = update_coordinates(r_pos, z_pos, theta_pos, joyJoystick, joyButtons)
                
                dof0, dof1, dof2 = calculate_positions(r_pos, z_pos, theta_pos)
                dof3 -= joyJoystick[5]*50
                dof4 -= (joyButtons[7] - joyButtons[6])*100
                dof5 = 6000
                dof5 += (joyButtons[0] - joyButtons[2])*500
                dof3 = np.uint16(dof3)
                dof4 = np.uint16(dof4)
                dof5 = np.uint16(dof5)          

                dof0 = np.clip(dof0, dofBounds[0][0], dofBounds[0][1])
                dof1 = np.clip(dof1, dofBounds[1][0], dofBounds[1][1])
                dof2 = np.clip(dof2, dofBounds[2][0], dofBounds[2][1])
                dof3 = np.clip(dof3, dofBounds[3][0], dofBounds[3][1])
                dof4 = np.clip(dof4, dofBounds[4][0], dofBounds[4][1])
                dof5 = np.clip(dof5, dofBounds[5][0], dofBounds[5][1])
                pub_thread.update(dof0, dof1, dof2, dof3, dof4, dof5)
                if(joyButtons[3] == 1):
                    pub_thread.stop()
                    r_pos = 12.8  # r, z, theta values measured from dof0 to dof3 joint
                    z_pos = 10.0

                print(f"r = {r_pos}")
                print(f"z = {z_pos} \n")
            Hand_Count +=1  # Increment count every cycle


    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
