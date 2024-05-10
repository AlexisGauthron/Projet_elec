#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>



int val1 = 0;
int val2 = 0;
int servo1 = 10;
int servo2 = 11;
int addition = 1;

Servo myservo;
Servo myservo1;
Adafruit_MPU6050 mpu;


// put function declarations here:
void readMPU();

void setup() {
  myservo.attach (10);
  myservo1.attach (11);
  Serial.begin (9600);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // put your setup code here, to run once:
}

void loop() {

  val1 +=addition;
  if (val1>360){
    //addition = -1;
  }else if (val1<0){
  //  addition = 1;
  }
  val2 -=addition;
  myservo.write (val1);
  myservo1.write (val2);

  readMPU ();  
}

// put function definitions here:

void readMPU(){ 
  ////Read acceleromter data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  
  if (g.gyro.z>0){
    addition = 1;
  } else addition = -1;
  delay (500);
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println("°C");
}
/*
#include "Wire.h"
#include <Arduino.h>


const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265; int maxVal=402;

double x; double y; double z;

void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
}
void loop(){
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);

x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

Serial.print("AngleX= "); Serial.println(x);

Serial.print("AngleY= "); Serial.println(y);

Serial.print("AngleZ= "); Serial.println(z); Serial.println("-----------------------------------------");
delay(400);
}
*/
