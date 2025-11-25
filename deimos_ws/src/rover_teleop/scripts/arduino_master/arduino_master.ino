// Combined ROS Arduino Mega Sketch for Drivetrain, Arm, and Science Module
#define ROS_SERIAL_BUFFER_SIZE 512
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
#include <PololuMaestro.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(18, 19); // TX1 (18), RX1 (19) on the board
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

// Function prototypes
void pwmCallback(const std_msgs::Float32MultiArray& msg);
void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg);
void scienceModuleCallback(const std_msgs::Float32MultiArray& msg);

// ROS Subscribers
ros::Subscriber<std_msgs::Float32MultiArray> sub_pwm("/motor_pwm", pwmCallback);
ros::Subscriber<std_msgs::Float32MultiArray> pwmSub("science_module", scienceModuleCallback);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo_positions", servo_cb);

// PWM Callback for drivetrain
void pwmCallback(const std_msgs::Float32MultiArray& msg) {
    if (msg.data_length >= 2) {
        left_motor.writeMicroseconds(msg.data[0]);
        right_motor.writeMicroseconds(msg.data[1]);
    }
}

// Servo control callback for arm
void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
    for (int i = 0; i < 6; i++) {
        maestro.setTarget(i, cmd_msg.data[i]);
    }
}

// Science module callback
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
    pinMode(LEFT_MOTOR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN, OUTPUT);
    left_motor.attach(LEFT_MOTOR_PIN);
    right_motor.attach(RIGHT_MOTOR_PIN);
    
    // Science module setup
    augerMotor.attach(AUGER_PIN);
    actuatorMotor.attach(ACTUATOR_PIN);
    augerMotor.writeMicroseconds(1500);
    actuatorMotor.writeMicroseconds(1500);
    
    // ROS setup
    nh.initNode();
    nh.subscribe(sub_pwm);
    nh.subscribe(sub);
    nh.subscribe(pwmSub);
}

void loop() {
    nh.spinOnce();
    delay(20);
}
