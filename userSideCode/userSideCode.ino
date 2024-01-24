/* Get tilt angles on X and Y, and rotation angle on Z
   Angles are given in degrees

   License: MIT
*/
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>

const int flexPinThumb = A11;    // jempol
const int flexPinIndex = A12;    // telunjuk


const int flexPinMiddle = A13;   // jari tengah
const int flexPinRing = A14;     // jari manis
const int flexPinPinky = A15;    // jari kelingking

ros::NodeHandle nh;

std_msgs::Float32 gyro1X;
std_msgs::Float32 gyro1Y;
std_msgs::Float32 gyro1Z;
std_msgs::Float32 gyro2X;
std_msgs::Float32 gyro2Y;
std_msgs::Float32 gyro2Z;
std_msgs::Float32 gyro3X;
std_msgs::Float32 gyro3Y;
std_msgs::Float32 gyro3Z;
std_msgs::Float32 ServoThumb;
std_msgs::Float32 ServoIndex;
std_msgs::Float32 ServoMiddle;
std_msgs::Float32 ServoRing;
std_msgs::Float32 ServoPinky;
ros::Publisher chatter_gyro1X("gyro1X", &gyro1X);
ros::Publisher chatter_gyro1Y("gyro1Y", &gyro1Y);
ros::Publisher chatter_gyro1Z("gyro1Z", &gyro1Z);
ros::Publisher chatter_gyro2X("gyro2X", &gyro2X);
ros::Publisher chatter_gyro2Y("gyro2Y", &gyro2Y);
ros::Publisher chatter_gyro2Z("gyro2Z", &gyro2Z);
ros::Publisher chatter_gyro3X("gyro3X", &gyro3X);
ros::Publisher chatter_gyro3Y("gyro3Y", &gyro3Y);
ros::Publisher chatter_gyro3Z("gyro3Z", &gyro3Z);
ros::Publisher chatter_ServoThumb("ServoThumb", &ServoThumb);
ros::Publisher chatter_ServoIndex("ServoIndex", &ServoIndex);
ros::Publisher chatter_ServoMiddle("ServoMiddle", &ServoMiddle);
ros::Publisher chatter_ServoRing("ServoRing", &ServoRing);
ros::Publisher chatter_ServoPinky("ServoPinky", &ServoPinky);

int gyro[] = { 2, 3, 4 };
int numSensors = sizeof(gyro) / sizeof(gyro[0]);

float normalizedCenteredAngle(float sensorAngle) {
  return 180 - 1 * sensorAngle - 90.00;
}
float normalizedAngle(float sensorAngle) {
  return 180 - sensorAngle;
}

float invertededCenteredAngle(float sensorAngle) {
  return  sensorAngle + 90.00;
}

float minToPlus(float sensorAngle) {
  return  sensorAngle * -1;
}

MPU6050 mpu(Wire);
MPU6050 mpu2(Wire);
MPU6050 mpu3(Wire);
unsigned long timer = 0;

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(chatter_ServoThumb);
  nh.advertise(chatter_ServoIndex);
  nh.advertise(chatter_ServoMiddle);
  nh.advertise(chatter_ServoRing);
  nh.advertise(chatter_ServoPinky);
  nh.advertise(chatter_gyro1X);
  nh.advertise(chatter_gyro1Y);
  nh.advertise(chatter_gyro1Z);
  nh.advertise(chatter_gyro2X);
  nh.advertise(chatter_gyro2Y);
  nh.advertise(chatter_gyro2Z);
  nh.advertise(chatter_gyro3X);
  nh.advertise(chatter_gyro3Y);
  nh.advertise(chatter_gyro3Z);

  for (int i = 0; i < numSensors; i++) {
    pinMode(gyro[i], OUTPUT);
    if (i == 0) {
      digitalWrite(gyro[i], LOW);
    } else {
      digitalWrite(gyro[i], HIGH);
    }
  }
  Wire.begin();

  byte status = mpu.begin();
  for (int i = 0; i < numSensors; i++) {
    digitalWrite(gyro[i], LOW);
    if (i == 0) {
      status = mpu.begin();
      Serial.print(F("MPU6050 status: "));
      Serial.println(status);
      while (status != 0) {}  // stop everything if could not connect to MPU6050

      Serial.println(F("Calculating offsets, do not move MPU6050"));
      delay(1000);
      // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
      mpu.calcOffsets();  // gyro and accelero
      Serial.println("Done!\n");
    }
    if (i == 1) {
      status = mpu2.begin();
      Serial.print(F("MPU6050 status: "));
      Serial.println(status);
      while (status != 0) {}  // stop everything if could not connect to MPU6050

      Serial.println(F("Calculating offsets, do not move MPU6050"));
      delay(1000);
      // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
      mpu2.calcOffsets();  // gyro and accelero
      Serial.println("Done!\n");
    }
    if (i == 2) {
      status = mpu3.begin();
      Serial.print(F("MPU6050 status: "));
      Serial.println(status);
      while (status != 0) {}  // stop everything if could not connect to MPU6050

      Serial.println(F("Calculating offsets, do not move MPU6050"));
      delay(1000);
      // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
      mpu3.calcOffsets();  // gyro and accelero
      Serial.println("Done!\n");
    }
    digitalWrite(gyro[i], HIGH);
  }
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    int flexPositionThumb = analogRead(flexPinThumb);
    int servoPositionThumb = map(flexPositionThumb, 753, 820, 180, 0);
    servoPositionThumb = constrain(servoPositionThumb, 0, 180);

    // Repeat the above process for other flex sensors and servos...
    int flexPositionIndex = analogRead(flexPinIndex);
    int servoPositionIndex = map(flexPositionIndex, 770, 885, 180, 0);
    servoPositionIndex = constrain(servoPositionIndex, 0, 180);

    int flexPositionMiddle = analogRead(flexPinMiddle);
    int servoPositionMiddle = map(flexPositionMiddle, 755, 852, 180, 0);
    servoPositionMiddle = constrain(servoPositionMiddle, 0, 180);

    int flexPositionRing = analogRead(flexPinRing);
    int servoPositionRing = map(flexPositionRing, 780, 856, 180, 0);
    servoPositionRing = constrain(servoPositionRing, 0, 180);

    int flexPositionPinky = analogRead(flexPinPinky);
    int servoPositionPinky = map(flexPositionPinky, 760, 865, 0, 180);
    servoPositionPinky = constrain(servoPositionPinky, 0, 180);
    
        ServoThumb.data = float(servoPositionThumb);
        ServoIndex.data = float(servoPositionIndex);
        ServoMiddle.data = float(servoPositionMiddle);
        ServoRing.data = float(servoPositionRing);
        ServoPinky.data = float(servoPositionPinky);
    chatter_ServoThumb.publish(&ServoThumb);
    chatter_ServoIndex.publish(&ServoIndex);
    chatter_ServoMiddle.publish(&ServoMiddle);
    chatter_ServoRing.publish(&ServoRing);
    chatter_ServoPinky.publish(&ServoPinky);
    nh.spinOnce();
    delay(25);
    digitalWrite(gyro[i], LOW);
    if (i == 0) {
      mpu.update();
      //      Serial.println("Gyro 1 : ");
      //      Serial.print("X : ");
      //      Serial.print(mpu.getAngleX());
      //      Serial.print("\tY : ");
      //      Serial.print(mpu.getAngleY());
      //      Serial.print("\tZ : ");
      //      Serial.println(mpu.getAngleZ());
      gyro1X.data = normalizedCenteredAngle(mpu.getAngleX());
      gyro1Y.data = normalizedCenteredAngle(mpu.getAngleY());
      gyro1Z.data = 179.11;
      chatter_gyro1X.publish(&gyro1X);
      chatter_gyro1Y.publish(&gyro1Y);
      chatter_gyro1Z.publish(&gyro1Z);
      nh.spinOnce();
    }
    if (i == 1) {
      mpu2.update();
      Serial.println("Gyro 2 : ");
      Serial.print("X : ");
      Serial.print(mpu2.getAngleX());
      Serial.print("\tY : ");
      Serial.print(mpu2.getAngleY());
      Serial.print("\tZ : ");
      Serial.println(mpu2.getAngleZ());
      gyro2X.data = normalizedCenteredAngle(mpu2.getAngleX());
      gyro2Y.data = minToPlus(mpu2.getAngleY());
      gyro2Z.data = normalizedCenteredAngle(mpu2.getAngleZ());
      chatter_gyro2X.publish(&gyro2X);
      chatter_gyro2Y.publish(&gyro2Y);
      chatter_gyro2Z.publish(&gyro2Z);
      nh.spinOnce();
    }
    if (i == 2) {
      mpu3.update();
      //      Serial.println("Gyro 3 : ");
      //      Serial.print("X : ");
      //      Serial.print(mpu3.getAngleX());
      //      Serial.print("\tY : ");
      //      Serial.print(mpu3.getAngleY());
      //      Serial.print("\tZ : ");
      //      Serial.println(mpu3.getAngleZ());
      gyro3X.data = (normalizedCenteredAngle(mpu3.getAngleX()) + 40) - gyro2X.data + 90;
      gyro3Y.data = mpu3.getAngleY();
      gyro3Z.data = normalizedCenteredAngle(mpu3.getAngleZ()) - gyro2Z.data + 90;
      chatter_gyro3X.publish(&gyro3X);
      chatter_gyro3Y.publish(&gyro3Y);
      chatter_gyro3Z.publish(&gyro3Z);
      nh.spinOnce();
    }
    delay(25);
    digitalWrite(gyro[i], HIGH);
  }
}
