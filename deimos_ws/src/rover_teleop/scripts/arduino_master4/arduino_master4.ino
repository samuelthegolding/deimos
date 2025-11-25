// Combined ROS Arduino Mega Sketch with Stepper Motor Control
#define ROS_SERIAL_BUFFER_SIZE 512
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <PololuMaestro.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(18, 19); // TX1 (18), RX1 (19)
#endif

MicroMaestro maestro(maestroSerial);
ros::NodeHandle nh;

// ======================= LED Tower ==========================
#define GREEN_PIN 2
#define YELLOW_PIN 3
#define RED_PIN 4

String current_status = "";
unsigned long last_flash_time = 0;
bool green_led_state = false;

// ======================= Drivetrain ==========================
Servo left_motor;
Servo right_motor;
#define LEFT_MOTOR_PIN 6
#define RIGHT_MOTOR_PIN 7

// ======================= Science Module ======================
Servo augerMotor;
Servo actuatorMotor;
#define AUGER_PIN 10
#define ACTUATOR_PIN 11

// ======================= Stepper Motor Pins ==================
#define SYRINGE_STEP_PIN 8
#define SYRINGE_DIR_PIN 9
#define CAROUSEL_STEP_PIN 12
#define CAROUSEL_DIR_PIN 13

// ======================= Stepper Motor Params ================
const int stepsPerRev = 200;
const int microsteps = 16;
const int stepDelayMicros = 500;

long syringeCurrentSteps = 0;
long carouselCurrentSteps = 0;

// ======================= Parameters ==========================
double wheel_base = 0.92;
double max_linear_vel = 0.5;
double max_angular_vel = 2.0;
int min_pwm = 1000;
int neutral_pwm = 1500;
int max_pwm = 2000;
float deadzone = 0.05;
unsigned long last_cmd_time = 0;
unsigned long cmd_timeout_ms = 300;

// ======================= Function Prototypes =================
void twistCallback(const geometry_msgs::Twist& cmd_vel);
void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg);
void scienceModuleCallback(const std_msgs::Float32MultiArray& msg);
void syringeAngleCallback(const std_msgs::Float32& msg);
void carouselAngleCallback(const std_msgs::Float32& msg);
void towerStatusCallback(const std_msgs::String& msg);

void setMotorPWM(double left_vel, double right_vel);
void setNeutralPWM();
int angleToSteps(float angle);
void moveStepper(int stepPin, int dirPin, long& currentStepCount, float targetAngle);

// ======================= ROS Subscribers =====================
ros::Subscriber<geometry_msgs::Twist> sub_twist("/cmd_vel", twistCallback);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo_positions", servo_cb);
ros::Subscriber<std_msgs::Float32MultiArray> pwmSub("science_module", scienceModuleCallback);
ros::Subscriber<std_msgs::Float32> syringeSub("syringe_angle", syringeAngleCallback);
ros::Subscriber<std_msgs::Float32> carouselSub("carousel_angle", carouselAngleCallback);
ros::Subscriber<std_msgs::String> towerSub("tower_status", towerStatusCallback);

// ======================= Setup ===============================
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

  pinMode(SYRINGE_STEP_PIN, OUTPUT);
  pinMode(SYRINGE_DIR_PIN, OUTPUT);
  pinMode(CAROUSEL_STEP_PIN, OUTPUT);
  pinMode(CAROUSEL_DIR_PIN, OUTPUT);

  pinMode(GREEN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);

  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(RED_PIN, LOW);

  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub);
  nh.subscribe(pwmSub);
  nh.subscribe(syringeSub);
  nh.subscribe(carouselSub);
  nh.subscribe(towerSub);
}

// ======================= Loop ================================
void loop() {
  nh.spinOnce();

  if (millis() - last_cmd_time > cmd_timeout_ms) {
    setNeutralPWM();
  }

  delay(5);

  // LED tower control
  unsigned long now = millis();

  if (current_status == "autonomous") {
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(YELLOW_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
  } else if (current_status == "manual") {
    digitalWrite(RED_PIN, LOW);
    digitalWrite(YELLOW_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
  } else if (current_status == "arrived") {
    digitalWrite(RED_PIN, LOW);
    digitalWrite(YELLOW_PIN, LOW);

    if (now - last_flash_time > 500) {
      green_led_state = !green_led_state;
      digitalWrite(GREEN_PIN, green_led_state ? HIGH : LOW);
      last_flash_time = now;
    }
  } else {
    digitalWrite(RED_PIN, LOW);
    digitalWrite(YELLOW_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
  }
}

// ======================= Callback Implementations ============

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
  right_pwm = constrain(2 * neutral_pwm - right_pwm, min_pwm, max_pwm);

  left_motor.writeMicroseconds(left_pwm);
  right_motor.writeMicroseconds(right_pwm);
}

void setNeutralPWM() {
  left_motor.writeMicroseconds(neutral_pwm);
  right_motor.writeMicroseconds(neutral_pwm);
}

void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, cmd_msg.data[i]);
  }
}

void scienceModuleCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length == 2) {
    augerMotor.writeMicroseconds(constrain(msg.data[0], 1000, 2000));
    actuatorMotor.writeMicroseconds(constrain(msg.data[1], 1000, 2000));
  }
}

int angleToSteps(float angle) {
  return (int)((stepsPerRev * microsteps) * (angle / 360.0));
}

void moveStepper(int stepPin, int dirPin, long& currentStepCount, float targetAngle) {
  long targetSteps = angleToSteps(targetAngle);
  long deltaSteps = targetSteps - currentStepCount;
  bool dir = deltaSteps >= 0;
  digitalWrite(dirPin, dir ? HIGH : LOW);

  for (long i = 0; i < abs(deltaSteps); i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayMicros);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayMicros);
  }

  currentStepCount = targetSteps;
}

void syringeAngleCallback(const std_msgs::Float32& msg) {
  moveStepper(SYRINGE_STEP_PIN, SYRINGE_DIR_PIN, syringeCurrentSteps, msg.data);
}

void carouselAngleCallback(const std_msgs::Float32& msg) {
  moveStepper(CAROUSEL_STEP_PIN, CAROUSEL_DIR_PIN, carouselCurrentSteps, msg.data);
}

void towerStatusCallback(const std_msgs::String& msg) {
  current_status = msg.data;
}
