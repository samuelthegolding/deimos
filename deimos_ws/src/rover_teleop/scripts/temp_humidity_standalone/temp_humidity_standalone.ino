#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

std_msgs::Float32 humidity_msg;
ros::Publisher pub_humidity("humidity", &humidity_msg);

#define TEMP_PIN A0
#define HUMID_PIN A1
#define TEMP_SLOPE 0.04   // Slope and intercept data from calibration sheet
#define TEMP_INTERCEPT -40
#define HUMID_SLOPE 0.04
#define HUMID_INTERCEPT 0.00

void setup() {
  nh.initNode();
  nh.advertise(pub_temp);
  nh.advertise(pub_humidity);
  Serial.begin(57600); // Set the baud rate to 57600 for ROS
  Serial.println("ROS Temperature & Humidity Publisher Initialized (57600 baud)");
}

void loop() {
  int tempRaw = analogRead(TEMP_PIN);  // Get voltage from analog pins
  int humidRaw = analogRead(HUMID_PIN);

  float tempVoltage = tempRaw * (5.0 / 1023.0);
  float humidVoltage = humidRaw * (5.0 / 1023.0);

  float temperatureC = (tempVoltage * 1000) * TEMP_SLOPE + TEMP_INTERCEPT;
  float humidity = (humidVoltage * 1000) * HUMID_SLOPE + HUMID_INTERCEPT;

  temp_msg.data = temperatureC;
  pub_temp.publish(&temp_msg);

  humidity_msg.data = humidity;
  pub_humidity.publish(&humidity_msg);

  nh.spinOnce();
  delay(100); // Publish at 10 Hz (adjust as needed)
}
