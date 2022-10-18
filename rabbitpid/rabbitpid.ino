#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int request = 0;

const int calibrationValue = 40;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

const int sensorPinLL = A3;
const int sensorPinCL = A2;
const int sensorPinCR = A1;
const int sensorPinRR = A0;

int sensors;

float Kp = .3;
float Ki = 0;
float Kd = .5;

bool on;

unsigned long startTime, elapsedTime;

int P, I, D;

int lastError = 0;

uint8_t maxspeedraw = 50;
uint8_t basespeedraw = 30;
uint8_t maxspeed, basespeed;

String output = "";

void setup()
{
    AFMS.begin();
    //
    // start the serial port
    //
    long baudRate = 9600;   // NOTE1: The baudRate for sending & receiving programs must match
    Serial.begin(baudRate); // NOTE2: Set the baudRate to 115200 for faster communication
    Serial.setTimeout(1);
    
    pinMode(sensorPinLL, INPUT);
    pinMode(sensorPinCL, INPUT);
    pinMode(sensorPinCR, INPUT);
    pinMode(sensorPinRR, INPUT);

    motorL->run(RELEASE);
    motorR->run(RELEASE);

    // Only continue while serial connection is available
    // This means nothing happens until the Python program is run
    while (!Serial.available())
    {
    }

    while (request == 0)
    {
      request = Serial.readString().toInt();
    }
    
    startTime = millis();
}


void loop()
{
//    if (Serial.available() > 0)
//    {
//      request = Serial.readString().toInt();
//    }

    if (request != 0)
    {
      maxspeed = maxspeedraw * request;
      basespeed = basespeedraw * request;

      PID_control();
    }
    else {
      motorL->run(RELEASE);
      motorR->run(RELEASE);
      exit;
    }
}

void PID_control()
{
  uint16_t position = sensorValue();
  
  int error = 500 - position;
  
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P*Kp + I*Ki + D*Kd;

  int motorspeedL = basespeed + motorspeed;
  int motorspeedR = basespeed - motorspeed;


  if (motorspeedL > maxspeed) {
    motorspeedL = maxspeed;
  }
  if (motorspeedR > maxspeed) {
    motorspeedR = maxspeed;
  }
  if (motorspeedL < 0) {
    motorspeedL = 0;
  }
  if (motorspeedR < 0) {
    motorspeedR = 0;
  }

  motorL->setSpeed(motorspeedL);
  motorR->setSpeed(motorspeedR);
  motorL->run(FORWARD);
  motorR->run(FORWARD);

  output = output + motorspeedL + "," + motorspeedR;
  Serial.println(output);
}

uint16_t sensorValue()
{
  float rawLL = analogRead(sensorPinLL);
  float rawCL = analogRead(sensorPinCL);
  float rawCR = analogRead(sensorPinCR);
  float rawRR = analogRead(sensorPinRR);
  bool sensorLL = onLine(rawLL);
  bool sensorCL = onLine(rawCL);
  bool sensorCR = onLine(rawCR);
  bool sensorRR = onLine(rawRR);
  output = String(float(millis() - startTime)) + "," + rawLL + "," + rawCL + "," + rawCR + "," + rawRR + ",";
//  Serial.print(",");
//  Serial.print(rawLL);
//  Serial.print(",");
//  Serial.print(rawCL);
//  Serial.print(",");
//  Serial.print(rawCR);
//  Serial.print(",");
//  Serial.print(rawRR);
//  Serial.print(",");

  if (sensorLL && !sensorCL && !sensorCR && !sensorRR)
  {
    return 800;
  }
  else if (sensorLL && sensorCL && !sensorCR && !sensorRR)
  {
    return 700;
  }
  else if (!sensorLL && sensorCL && !sensorCR && !sensorRR)
  {
    return 600;
  }
  else if (!sensorLL && sensorCL && sensorCR && !sensorRR)
  {
    return 500;
  }
  else if (!sensorLL && !sensorCL && sensorCR && !sensorRR)
  {
    return 400;
  }
  else if (!sensorLL && !sensorCL && sensorCR && sensorRR)
  {
    return 300;
  }
  else if (!sensorLL && !sensorCL && !sensorCR && sensorRR)
  {
    return 200;
  }
}

bool onLine(float sensorRaw)
{
    if (sensorRaw > calibrationValue) // Add calibration stuff here
    {
        return true;
    }
    else
    {
        return false;
    }
}
