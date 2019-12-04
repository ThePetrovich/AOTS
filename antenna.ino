#define AZIMUTH_MODE 1 // 1 - in case of using azimuth angless; 2 - in case of using default angles
#define lowerServo 5
#define higherServo 6
#define upButton 1
#define downButton 3
#define leftButton 4 
#define rightButton 7
#define LED 8
#define RXPin 9
#define TXPin 10
#define GPSBaud 9600

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include "GyverButton.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_9DOF.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

TinyGPSPlus gps;
TinyGPSCustom mdecl(gps, "GPRMC", 11);
TinyGPSCustom mdeclDir(gps, "GPRMC", 12);
SoftwareSerial ss(RXPin, TXPin);

Adafruit_9DOF dof   = Adafruit_9DOF();
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

GButton upB(upButton, LOW_PULL, NORM_OPEN);
GButton downB(downButton, LOW_PULL, NORM_OPEN);
GButton leftB(leftButton, LOW_PULL, NORM_OPEN);
GButton rightB(rightButton, LOW_PULL, NORM_OPEN);

GButton setStepTimeout(30);

volatile int sm = 0;
String str = ""; 
volatile bool stringComplete = false;
volatile bool st = false;
int alpha = 0, beta = 0;
int tAlpha = 0, tBeta = 0;
int offsetAlpha = 0, offsetBeta = 0;
float declinationAngle = 0;

void change_st()
{
  st = !st;
}

void getOrientation(){
  sensors_event_t accel_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    roll = orientation.roll;
    pitch = orientation.pitch;
  }
}

float getMagneticDeclination(){
  float  mdeclDirFloat = 0;
  if (mdecl.isUpdated() || mdeclDir.isUpdated())
  {
    String mDeclTemp = mdecl.value();
    mdeclDirFloat = mDeclTemp.toFloat();
    String magDeclDirTemp = mdeclDir.value();
    if(magDeclDirTemp == "W") mdeclDirFloat = -mdeclDirFloat;
  }
  return mdeclDirFloat;
}

void updateGPS(){
    while (ss.available() > 0)
        gps.encode(ss.read());
}

void setup()
{
  pinMode(LED, OUTPUT);
  mag.begin();
  Serial.begin(9600);
  str.reserve(200);
  attachInterrupt(0, change_st, RISING);
  Serial.begin(9600);
  ss.begin(GPSBaud);
}

void loop() 
{
  getOrientation();
  declinationAngle = getMagneticDeclination();
  updateGPS();  
  if (st)
  {
    digitalWrite(LED, HIGH);
    if (upB.isClick())
      offsetBeta++;
    if (downB.isClick())
      offsetBeta--;
    if (leftB.isClick())
      offsetAlpha--;
    if (rightB.isClick())
      offsetAlpha++;
    if (upB.isStep())
      offsetBeta++;
    if (downB.isStep())
      offsetBeta--;
    if (leftB.isStep())
      offsetAlpha--;
    if (rightB.isStep())
      offsetAlpha++;
  }
  else
  {
    digitalWrite(LED, LOW);
    if (stringComplete) 
    {
      get_angles(str);
      str = "";
      stringComplete = false;
      sm = 0;
    }
  }
  alpha = offsetAlpha + tAlpha;
  if (AZIMUTH_MODE)
    alpha += get_heading();
  beta = offsetBeta + tBeta;

  while (alpha > 360)
    alpha -= 360;
  if (alpha > 180)
  {
    alpha -= 180;
    beta = 180 - beta;
  }
  if (alpha < 0)
  {
    alpha = 180 - abs(alpha);
    beta = 180 - beta;
  }

  if (beta > 180)
    beta = 180;
  if (beta < 0)
    beta = 0;
  Serial.print("ALPHA: "); Serial.print(alpha); Serial.print(" BETA: "); Serial.println(beta);
  
}

int get_heading()
{
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI;
  Serial.println(headingDegrees);
  return (int)headingDegrees;
}

void get_angles(String s)
{
   tAlpha = 0; tBeta = 0;
   String temp = "";
   for (int i = 0; i < s.length(); i++)
      {
        if (s[i] == '=')
        {
          if (s[i - 1] == 'A')
          {
            while (s[i + 1] != ';')
              {
              temp += s[i + 1];
              i++;
              }
            tAlpha = atoi(temp.c_str());
            temp = "";
          }
        }
        if (s[i] == '=')
        {
          if (s[i - 1] == 'B')
          {
            while (s[i + 1] != ';')
              {
              temp += s[i + 1];
              i++;
              }
            tBeta = atoi(temp.c_str());
            temp = "";
          }
        }
      }
}

void serialEvent() 
{
  while (Serial.available() && !st) 
  {
    char inChar = (char)Serial.read();
    str += inChar;
    if (inChar == ';') 
      sm++;
    if (sm == 2)
        stringComplete = true;
  }
}
