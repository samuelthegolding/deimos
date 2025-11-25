#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
 
ros::NodeHandle nh;
 
// Define two Servo objects for controlling the motors
Servo left_motor;
Servo right_motor;
 
// Define a variable to store the incoming PWM values
std_msgs::Float32MultiArray pwm_msg;
 
// Callback function to handle incoming PWM values from ROS
void pwmCallback(const std_msgs::Float32MultiArray& msg) {
  // Assign PWM values for left and right motors from the incoming message
  int pwm_left = msg.data[0];  // First value is for the left motor
  int pwm_right = msg.data[1]; // Second value is for the right motor
 
  // Write the PWM values to the motors (make sure to map to valid PWM range)
  left_motor.writeMicroseconds(pwm_left);  // Send PWM value to the left motor
  right_motor.writeMicroseconds(pwm_right); // Send PWM value to the right motor
}
 
ros::Subscriber<std_msgs::Float32MultiArray> sub_pwm("/motor_pwm", pwmCallback);
 
void setup() {
  // Start the serial communication and ROS node
  Serial.begin(57600);  // Set baud rate to match rosserial
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  // Attach the motors to specific pins (pins 9 and 10 are just examples)
  left_motor.attach(9);  // Attach left motor to pin 9
  right_motor.attach(10); // Attach right motor to pin 10
 
  // Initialize ROS node
  nh.initNode();
 
  // Subscribe to the /motor_pwm topic to receive PWM commands
  nh.subscribe(sub_pwm);
}
 
void loop() {
  // Handle incoming ROS messages
  nh.spinOnce();  // Process incoming messages
  delay(20);      // Small delay to avoid overwhelming the system
}
