#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// Define motor control objects
Servo augerMotor;      // Brushless motor ESC
Servo actuatorMotor;   // Linear actuator

// Define PWM pin connections
#define AUGER_PIN 9      // PWM pin for auger motor ESC
#define ACTUATOR_PIN 10  // PWM pin for linear actuator

// Callback function to receive PWM values from ROS
void pwmCallback(const std_msgs::Float32MultiArray& msg) {
    if (msg.data_length == 2) {
        int augerPWM = constrain(msg.data[0], 1000, 2000);    // Auger motor PWM
        int actuatorPWM = constrain(msg.data[1], 1000, 2000); // Actuator PWM

        augerMotor.writeMicroseconds(augerPWM);
        actuatorMotor.writeMicroseconds(actuatorPWM);
    }
}

// ROS node setup
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> pwmSub("science_module", &pwmCallback);

void setup() {
    nh.initNode();
    nh.subscribe(pwmSub);

    // Attach motors to their respective pins
    augerMotor.attach(AUGER_PIN);
    actuatorMotor.attach(ACTUATOR_PIN);

    // Set initial stop positions (1500 µs is neutral for many ESCs and servos)
    augerMotor.writeMicroseconds(1500);
    actuatorMotor.writeMicroseconds(1500);
}

void loop() {
    nh.spinOnce(); // Process ROS messages
    delay(10); // Small delay to allow smooth operation
}
