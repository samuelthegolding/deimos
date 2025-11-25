#define ROS_SERIAL_BUFFER_SIZE 512
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Servo.h>
#include <PololuMaestro.h>

// Maestro Serial Setup
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(18, 19); // TX1 (18), RX1 (19)
#endif
MicroMaestro maestro(maestroSerial);
ros::NodeHandle nh;

// Drivetrain Motors
Servo left_motor;
Servo right_motor;
#define LEFT_MOTOR_PIN 6
#define RIGHT_MOTOR_PIN 7

// Science Module
Servo augerMotor;
Servo actuatorMotor;
#define AUGER_PIN 10
#define ACTUATOR_PIN 11

// Camera Servo (standard PWM, not Maestro)
Servo cameraServo;
#define CAMERA_SERVO_PIN 5  // Adjust if needed

// Stepper Pins
#define SYRINGE_STEP_PIN 8
#define SYRINGE_DIR_PIN 9
#define CAROUSEL_STEP_PIN 12
#define CAROUSEL_DIR_PIN 13
const int stepDelayMicros = 100;

// Temp & Humidity
#define TEMP_PIN A0
#define HUMID_PIN A1
#define TEMP_SLOPE 0.04
#define TEMP_INTERCEPT -40
#define HUMID_SLOPE 0.04
#define HUMID_INTERCEPT 0.0
std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);
std_msgs::Float32 humidity_msg;
ros::Publisher pub_humidity("humidity", &humidity_msg);

// Drivetrain Parameters
double wheel_base = 0.92;
double max_linear_vel = 0.5;
double max_angular_vel = 2.0;
int min_pwm = 1000;
int neutral_pwm = 1500;
int max_pwm = 2000;
float deadzone = 0.05;
unsigned long last_cmd_time = 0;
unsigned long cmd_timeout_ms = 300;

// ROS Callbacks
void twistCallback(const geometry_msgs::Twist& cmd_vel);
void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg);
void scienceModuleCallback(const std_msgs::Float32MultiArray& msg);
void syringeStepCallback(const std_msgs::Int32& msg);
void carouselStepCallback(const std_msgs::Int32& msg);
void cameraServoCallback(const std_msgs::UInt16MultiArray& angle_msg);  // NEW

// ROS Interfaces
ros::Subscriber<geometry_msgs::Twist> sub_twist("/cmd_vel", twistCallback);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo_positions", servo_cb);
ros::Subscriber<std_msgs::Float32MultiArray> pwmSub("science_module", scienceModuleCallback);
ros::Subscriber<std_msgs::Int32> syringeSub("/syringe_count", syringeStepCallback);
ros::Subscriber<std_msgs::Int32> carouselSub("/carousel_count", carouselStepCallback);
ros::Subscriber<std_msgs::UInt16MultiArray> camServoSub("camera_servo_angle", cameraServoCallback);  // NEW

// === Setup ===
void setup() {
  Serial.begin(57600);
  maestroSerial.begin(9600);

  left_motor.attach(LEFT_MOTOR_PIN);
  right_motor.attach(RIGHT_MOTOR_PIN);
  setNeutralPWM();

  augerMotor.attach(AUGER_PIN);
  actuatorMotor.attach(ACTUATOR_PIN);
  augerMotor.writeMicroseconds(1500);
  actuatorMotor.writeMicroseconds(1500);

  cameraServo.attach(CAMERA_SERVO_PIN);     // NEW
  cameraServo.write(67);                    // NEW: center position

  pinMode(SYRINGE_STEP_PIN, OUTPUT);
  pinMode(SYRINGE_DIR_PIN, OUTPUT);
  pinMode(CAROUSEL_STEP_PIN, OUTPUT);
  pinMode(CAROUSEL_DIR_PIN, OUTPUT);

  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub);
  nh.subscribe(pwmSub);
  nh.subscribe(syringeSub);
  nh.subscribe(carouselSub);
  nh.subscribe(camServoSub);  // NEW
  nh.advertise(pub_temp);
  nh.advertise(pub_humidity);
}

// === Main Loop ===
void loop() {
  nh.spinOnce();

  // Safety timeout for drivetrain
  if (millis() - last_cmd_time > cmd_timeout_ms) {
    setNeutralPWM();
  }

  // Temp & Humidity Publishing
  int tempRaw = analogRead(TEMP_PIN);
  int humidRaw = analogRead(HUMID_PIN);
  float tempVoltage = tempRaw * (5.0 / 1023.0);
  float humidVoltage = humidRaw * (5.0 / 1023.0);
  float temperatureC = (tempVoltage * 1000) * TEMP_SLOPE + TEMP_INTERCEPT;
  float humidity = (humidVoltage * 1000) * HUMID_SLOPE + HUMID_INTERCEPT;
  temp_msg.data = temperatureC;
  humidity_msg.data = humidity;
  pub_temp.publish(&temp_msg);
  pub_humidity.publish(&humidity_msg);

  delay(100); // Loop at 10 Hz
}

// === Drivetrain ===
void twistCallback(const geometry_msgs::Twist& cmd_vel) {
  last_cmd_time = millis();
  double linear = (abs(cmd_vel.linear.x) < deadzone) ? 0.0 : cmd_vel.linear.x;
  double angular = (abs(cmd_vel.angular.z) < deadzone) ? 0.0 : cmd_vel.angular.z;
  linear = constrain(linear, -max_linear_vel, max_linear_vel);
  angular = constrain(angular, -max_angular_vel, max_angular_vel);
  double left_vel = linear - angular * wheel_base / 2.0;
  double right_vel = linear + angular * wheel_base / 2.0;
  setMotorPWM(left_vel, right_vel);
}

void setMotorPWM(double left_vel, double right_vel) {
  double max_wheel_vel = max_linear_vel + max_angular_vel * wheel_base / 2.0;
  int left_pwm = map(left_vel * 1000, -max_wheel_vel * 1000, max_wheel_vel * 1000, min_pwm, max_pwm);
  int right_pwm = map(right_vel * 1000, -max_wheel_vel * 1000, max_wheel_vel * 1000, min_pwm, max_pwm);
  left_pwm = constrain(left_pwm, min_pwm, max_pwm);
  right_pwm = constrain(2 * neutral_pwm - right_pwm, min_pwm, max_pwm); // invert
  left_motor.writeMicroseconds(left_pwm);
  right_motor.writeMicroseconds(right_pwm);
}

void setNeutralPWM() {
  left_motor.writeMicroseconds(neutral_pwm);
  right_motor.writeMicroseconds(neutral_pwm);
}

// === Maestro 6-Servo Arm ===
void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, cmd_msg.data[i]);
  }
}

// === Science Module (auger + actuator) ===
void scienceModuleCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length == 2) {
    augerMotor.writeMicroseconds(constrain(msg.data[0], 1000, 2000));
    actuatorMotor.writeMicroseconds(constrain(msg.data[1], 1000, 2000));
  }
}

// === Stepper Motor Control ===
void moveStepperSteps(int stepPin, int dirPin, int stepCount) {
  bool dir = stepCount >= 0;
  digitalWrite(dirPin, dir ? HIGH : LOW);
  for (int i = 0; i < abs(stepCount); i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayMicros);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayMicros);
  }
}

void syringeStepCallback(const std_msgs::Int32& msg) {
  moveStepperSteps(SYRINGE_STEP_PIN, SYRINGE_DIR_PIN, msg.data);
}

void carouselStepCallback(const std_msgs::Int32& msg) {
  moveStepperSteps(CAROUSEL_STEP_PIN, CAROUSEL_DIR_PIN, msg.data);
}

// === Camera Servo Control ===
void cameraServoCallback(const std_msgs::UInt16MultiArray& angle_msg) {
  if (angle_msg.data_length > 0) {
    int angle = angle_msg.data[0];
    cameraServo.write(angle);  // Move camera servo to specified angle (0–180)
  }
}
