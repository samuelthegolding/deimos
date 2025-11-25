// Combined ROS Arduino Mega Sketch for Drivetrain, Arm, and Science Module
#define ROS_SERIAL_BUFFER_SIZE 512
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
#include <PololuMaestro.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/UInt16MultiArray.h"

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(18, 19); // TX1 (18), RX1 (19)
#endif

MicroMaestro maestro(maestroSerial);
ros::NodeHandle nh;

// Drivetrain
Servo left_motor;
Servo right_motor;
#define LEFT_MOTOR_PIN 6
#define RIGHT_MOTOR_PIN 7

// Science Module
Servo augerMotor;
Servo actuatorMotor;
#define AUGER_PIN 10
#define ACTUATOR_PIN 11

// Function Prototypes
void twistCallback(const geometry_msgs::Twist& cmd_vel);
void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg);
void scienceModuleCallback(const std_msgs::Float32MultiArray& msg);
void setMotorPWM(double left_vel, double right_vel);
void setNeutralPWM();

// ROS Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_twist("/cmd_vel", twistCallback);
ros::Subscriber<std_msgs::Float32MultiArray> pwmSub("science_module", scienceModuleCallback);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo_positions", servo_cb);

// Parameters
double wheel_base = 0.92;       // Distance between wheels (m)
double max_linear_vel = 0.5;    // Max linear velocity (m/s)
double max_angular_vel = 2.0;   // Max angular velocity (rad/s)
int min_pwm = 1000;
int neutral_pwm = 1500;
int max_pwm = 2000;

// Variables to hold last velocity commands
double last_linear_vel = 0.0;
double last_angular_vel = 0.0;

float deadzone = 0.05;                // Joystick deadzone
unsigned long last_cmd_time = 0;     // Last time /cmd_vel received
unsigned long cmd_timeout_ms = 300;  // Stop after 300 ms without input

// Twist callback: compute wheel velocities and set PWM
void twistCallback(const geometry_msgs::Twist& cmd_vel) {
  last_cmd_time = millis(); // Update time of last received cmd_vel

  // Apply deadzone
  double linear = (abs(cmd_vel.linear.x) < deadzone) ? 0.0 : cmd_vel.linear.x;
  double angular = (abs(cmd_vel.angular.z) < deadzone) ? 0.0 : cmd_vel.angular.z;

  // Clamp values
  linear = constrain(linear, -max_linear_vel, max_linear_vel);
  angular = constrain(angular, -max_angular_vel, max_angular_vel);

  // Update the last velocity values
  last_linear_vel = linear;
  last_angular_vel = angular;

  // Compute wheel velocities
  double left_vel = linear - angular * wheel_base / 2.0;
  double right_vel = linear + angular * wheel_base / 2.0;

  setMotorPWM(left_vel, right_vel);
}

// Convert wheel velocity to PWM and set motors
void setMotorPWM(double left_vel, double right_vel) {
  // Determine max possible wheel velocity
  double max_wheel_vel = max_linear_vel + max_angular_vel * wheel_base / 2.0;

  // Map from velocity to PWM
  int left_pwm = map(left_vel * 1000, -max_wheel_vel * 1000, max_wheel_vel * 1000, min_pwm, max_pwm);
  int right_pwm = map(right_vel * 1000, -max_wheel_vel * 1000, max_wheel_vel * 1000, min_pwm, max_pwm);

  // Constrain PWM values and handle mirrored motor directions if needed
  left_pwm = constrain(left_pwm, min_pwm, max_pwm);
  right_pwm = constrain(2 * neutral_pwm - right_pwm, min_pwm, max_pwm); // Inverted for mirrored mount

  left_motor.writeMicroseconds(left_pwm);
  right_motor.writeMicroseconds(right_pwm);
}

// Neutral stop
void setNeutralPWM() {
  left_motor.writeMicroseconds(neutral_pwm);
  right_motor.writeMicroseconds(neutral_pwm);
}

// Servo control for arm
void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, cmd_msg.data[i]);
  }
}

// Science module control
void scienceModuleCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length == 2) {
    augerMotor.writeMicroseconds(constrain(msg.data[0], 1000, 2000));
    actuatorMotor.writeMicroseconds(constrain(msg.data[1], 1000, 2000));
  }
}

void setup() {
  Serial.begin(57600);
  maestroSerial.begin(9600);

  // Drivetrain setup
  left_motor.attach(LEFT_MOTOR_PIN);
  right_motor.attach(RIGHT_MOTOR_PIN);
  setNeutralPWM();

  // Science module setup
  augerMotor.attach(AUGER_PIN);
  actuatorMotor.attach(ACTUATOR_PIN);
  augerMotor.writeMicroseconds(1500);
  actuatorMotor.writeMicroseconds(1500);

  // ROS setup
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub);
  nh.subscribe(pwmSub);
}

void loop() {
  nh.spinOnce();

  // Stop motors if no new command received in time
  if (millis() - last_cmd_time > cmd_timeout_ms) {
    setNeutralPWM();
  }

  delay(5); // Reduced delay for responsiveness
}
