#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>

ros::NodeHandle nh;

// Declare the Servo pin
int servoKepalaAtasBawahPin = 3;
int servoKepalaKananKiriPin = 2;
int servoBahuDepanBelakang1Pin = 4;
int servoBahuDepanBelakang2Pin = 5;

int servoBahuMemutarPin = 6;
int servoSikuNaikTurunPin = 7;
int servoSikuMemutarPin = 8;

int pneumaticBahuKeluarKedalamPin1 = 42;
int pneumaticBahuKeluarKedalamPin2 = 44;
const int ENA_PIN = 46;  // the Arduino pin connected to the EN1 pin L298N





float normalizedCenteredAngle(float sensorAngle) {
  return 180 - 1 * sensorAngle - 90.00;
}
float normalizedAngle(float sensorAngle) {
  return 180 - sensorAngle;
}

float minToPlusAngle(float sensorAngle) {
  return -1 * sensorAngle;
}

float invertededCenteredAngle(float sensorAngle) {
  return sensorAngle + 90.00;
}

MPU6050 mpu2(Wire);

// Declare the Servo pin
const float Kp = 1.0;  // Proportional gain

Servo servoKepalaAtasBawah;
Servo servoKepalaKananKiri;
Servo servoBahuDepanBelakang1;
Servo servoBahuDepanBelakang2;
Servo servoBahuMemutar;
Servo servoSikuNaikTurun;
Servo servoSikuMemutar;

Servo servoThumb;   // jempol
Servo servoIndex;   // telunjuk
Servo servoMiddle;  // jari tengah
Servo servoRing;    // jari manis
Servo servoPinky;   // jari kelingking

float gyro2x;
float gyro2y;
float gyro2z;

void gyro1X(const std_msgs::Float32& gyro1XAngle) {
  // Serial.println(gyro1XAngle.data);
  servoKepalaAtasBawah.write(gyro1XAngle.data);
}
void gyro1Y(const std_msgs::Float32& gyro1YAngle) {
  servoKepalaKananKiri.write(gyro1YAngle.data);
}
void gyro1Z(const std_msgs::Float32& gyro1ZAngle) {
  // servoIndex.write(gyro1ZAngle.data);
}
void gyro2X(const std_msgs::Float32& gyro2XAngle) {
  float angleDifference = (gyro2XAngle.data) - (gyro2y + 90);
  float servoTargetPosition = 90.0 + Kp * angleDifference;
  // Serial.print("servoTargetPosition : ");
  // Serial.println(servoTargetPosition);
  servoBahuDepanBelakang1.write(servoTargetPosition);
  servoBahuDepanBelakang2.write(servoTargetPosition);
}
void gyro2Y(const std_msgs::Float32& gyro2YAngle) {


  float angleDifference = (gyro2YAngle.data) - (gyro2x);
  float servoTargetPosition = Kp * angleDifference;
  // Serial.print("\tservoTargetPosition : ");
  // Serial.println(servoTargetPosition);
  if (servoTargetPosition >= 3) {
    // Serial.println("Extends");
    digitalWrite(pneumaticBahuKeluarKedalamPin1, HIGH);
    digitalWrite(pneumaticBahuKeluarKedalamPin2, LOW);
  } else if (servoTargetPosition <= -3) {
    // Serial.println("Retracks");
    digitalWrite(pneumaticBahuKeluarKedalamPin1, LOW);
    digitalWrite(pneumaticBahuKeluarKedalamPin2, HIGH);
  } else {
    // Serial.println("Stops");
    digitalWrite(pneumaticBahuKeluarKedalamPin1, LOW);
    digitalWrite(pneumaticBahuKeluarKedalamPin2, LOW);
  }
}
void gyro2Z(const std_msgs::Float32& gyro2ZAngle) {
  // servoBahuMemutar.write(gyro2ZAngle.data);
}
void gyro3X(const std_msgs::Float32& gyro3XAngle) {
  char result[8];
  dtostrf(gyro3XAngle.data, 6, 2, result);
  nh.loginfo("gyro3XAngle = ");
  nh.loginfo(result);
  // servoSikuNaikTurun.write(gyro3XAngle.data);
}
void gyro3Y(const std_msgs::Float32& gyro3YAngle) {
  // Engga kepake
  // servoSikuMemutar.write(gyro3YAngle.data);
}
void gyro3Z(const std_msgs::Float32& gyro3ZAngle) {
  // servoSikuMemutar.write(gyro3ZAngle.data);
}
void ServoThumbF(const std_msgs::Float32& ServoThumbValue) {
  char result[8];
  dtostrf(ServoThumbValue.data, 6, 2, result);
  nh.loginfo("ServoThumbF = ");
  nh.loginfo(result);
  servoThumb.write(ServoThumbValue.data);
}
void ServoIndexF(const std_msgs::Float32& ServoIndexValue) {
  servoIndex.write(ServoIndexValue.data);
}
void ServoMiddleF(const std_msgs::Float32& ServoMiddleValue) {
  servoMiddle.write(ServoMiddleValue.data);
}
void ServoRingF(const std_msgs::Float32& ServoRingValue) {
  servoRing.write(ServoRingValue.data);
}
void ServoPinkyF(const std_msgs::Float32& ServoPinkyValue) {
  servoPinky.write(ServoPinkyValue.data);
}

ros::Subscriber<std_msgs::Float32> subServoThumb("ServoThumb", ServoThumbF);
ros::Subscriber<std_msgs::Float32> subServoIndex("ServoIndex", ServoIndexF);
ros::Subscriber<std_msgs::Float32> subServoMiddle("ServoMiddle", ServoMiddleF);
ros::Subscriber<std_msgs::Float32> subServoRing("ServoRing", ServoRingF);
ros::Subscriber<std_msgs::Float32> subServoPinky("ServoPinky", ServoPinkyF);
ros::Subscriber<std_msgs::Float32> subgyro1XAngle("gyro1X", gyro1X);
ros::Subscriber<std_msgs::Float32> subgyro1YAngle("gyro1Y", gyro1Y);
ros::Subscriber<std_msgs::Float32> subgyro1ZAngle("gyro1Z", gyro1Z);
ros::Subscriber<std_msgs::Float32> subgyro2XAngle("gyro2X", gyro2X);
ros::Subscriber<std_msgs::Float32> subgyro2YAngle("gyro2Y", gyro2Y);
ros::Subscriber<std_msgs::Float32> subgyro2ZAngle("gyro2Z", gyro2Z);
ros::Subscriber<std_msgs::Float32> subgyro3XAngle("gyro3X", gyro3X);
ros::Subscriber<std_msgs::Float32> subgyro3YAngle("gyro3Y", gyro3Y);
ros::Subscriber<std_msgs::Float32> subgyro3ZAngle("gyro3Z", gyro3Z);


void setup() {
  Serial.begin(9600);
  Wire.begin();
  servoKepalaAtasBawah.attach(servoKepalaAtasBawahPin);
  servoKepalaKananKiri.attach(servoKepalaKananKiriPin);
  servoBahuDepanBelakang1.attach(servoBahuDepanBelakang1Pin);
  servoBahuDepanBelakang2.attach(servoBahuDepanBelakang2Pin);
  servoBahuMemutar.attach(servoBahuMemutarPin);
  servoSikuNaikTurun.attach(servoSikuNaikTurunPin);
  servoSikuMemutar.attach(servoSikuMemutarPin);
  servoThumb.attach(11);   // jempol
  servoPinky.attach(10);   //pinky
  servoIndex.attach(12);   // telunjuk
  servoMiddle.attach(13);  // jari tengah
  servoRing.attach(9);     // jari manis
  pinMode(ENA_PIN, OUTPUT);
  pinMode(pneumaticBahuKeluarKedalamPin1, OUTPUT);
  pinMode(pneumaticBahuKeluarKedalamPin2, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);
  nh.initNode();
  nh.subscribe(subServoThumb);
  nh.subscribe(subServoIndex);
  nh.subscribe(subServoMiddle);
  nh.subscribe(subServoRing);
  nh.subscribe(subServoPinky);
  nh.subscribe(subgyro1XAngle);
  nh.subscribe(subgyro1YAngle);
  nh.subscribe(subgyro1ZAngle);
  nh.subscribe(subgyro2XAngle);
  nh.subscribe(subgyro2YAngle);
  nh.subscribe(subgyro2ZAngle);
  nh.subscribe(subgyro3XAngle);
  nh.subscribe(subgyro3YAngle);
  nh.subscribe(subgyro3ZAngle);

  byte status = mpu2.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu2.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
  // put your setup code here, to run once:
  servoThumb.write(180);
  servoIndex.write(180);
  servoMiddle.write(180);
  servoRing.write(180);
  servoPinky.write(0);
}

void loop() {
  servoSikuNaikTurun.write(130);
  //  nh.loginfo("Program info");
  mpu2.update();
  // Serial.print("\tX : ");
  // Serial.print(mpu2.getAngleX());
  gyro2x = minToPlusAngle(mpu2.getAngleX());
  // Serial.print("\tY : ");
  // Serial.print(mpu2.getAngleY());
  gyro2y = mpu2.getAngleY();
  // Serial.print("\tZ : ");
  // Serial.println(mpu2.getAngleZ());
  nh.spinOnce();
  delay(50);
  // delay(10000);
  // servoKepalaAtasBawah.write(90);
  // servoKepalaKananKiri.write(90);
  // // servoBahuMemutar.write(90);
  // servoThumb.write(180);
  // servoIndex.write(180);
  // servoMiddle.write(180);
  // servoRing.write(180);
  // servoPinky.write(0);
  // // servoBahuMemutar.write(130);
  // servoSikuNaikTurun.write(40);
  // delay(2000);
  //   servoThumb.write(0);
  // servoIndex.write(0);
  // servoMiddle.write(0);
  // servoRing.write(0);
  // servoPinky.write(180);
  // delay(5000);
  // servoThumb.write(180);
  // servoIndex.write(180);
  // servoMiddle.write(180);
  // servoRing.write(180);
  // servoPinky.write(0);
  // delay(1000);
  // servoSikuMemutar.write(130);
  // float angleDifference = (-50) - (gyro2x);
  // float servoTargetPosition = Kp * angleDifference;
  // Serial.print("\tservoTargetPosition : ");
  // Serial.println(servoTargetPosition);
  // if (servoTargetPosition >= 3) {
  //   Serial.println("Extends");
  //   digitalWrite(pneumaticBahuKeluarKedalamPin1, HIGH);
  //   digitalWrite(pneumaticBahuKeluarKedalamPin2, LOW);
  // }
  // else if (servoTargetPosition <= -3) {
  //   Serial.println("Retracks");
  //   digitalWrite(pneumaticBahuKeluarKedalamPin1, LOW);
  //   digitalWrite(pneumaticBahuKeluarKedalamPin2, HIGH);
  // }
  // else {
  //   Serial.println("Stops");
  //   digitalWrite(pneumaticBahuKeluarKedalamPin1, LOW);
  //   digitalWrite(pneumaticBahuKeluarKedalamPin2, LOW);
  // }
  // float angleDifference = (90) - (gyro2y + 90);
  // float servoTargetPosition = 90.0 + Kp * angleDifference;
  // // Serial.print("servoTargetPosition : ");
  // // Serial.println(servoTargetPosition);
  // servoBahuDepanBelakang1.write(servoTargetPosition);
  // servoBahuDepanBelakang2.write(servoTargetPosition);
}