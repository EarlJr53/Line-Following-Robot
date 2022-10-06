#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int request = 1;

const int calibrationValue = 500;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

const int sensorPinLL = A0;
const int sensorPinCL = A1;
const int sensorPinC = A2;
const int sensorPinCR = A3;
const int sensorPinRR = A4;

int sensors;

float Kp = 0;
float Ki = 0;
float Kd = 0;

bool on;

int P, I, D;

int lastError = 0;

uint8_t maxspeed = 60;
uint8_t basespeed = 20;

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
    pinMode(sensorPinC, INPUT);
    pinMode(sensorPinCR, INPUT);
    pinMode(sensorPinRR, INPUT);

    // Only continue while serial connection is available
    // This means nothing happens until the Python program is run
    while (!Serial.available())
    {
    }
    
//    while (request == 0)
//    {
//        request = Serial.readString().toInt();
//    }

    request = Serial.readString().toInt();
}


void loop()
{
    if (request == 0)
    {
      on = false;
      request = Serial.readString().toInt();
    }
    else
    {
      on = true;
      maxspeed = maxspeed * request;
      basespeed = basespeed * request;
    }


    if (on)
    {
      PID_control();
    }
    else {
      motorL->run(RELEASE);
      motorR->run(RELEASE);
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
}

uint16_t sensorValue()
{
  bool sensorLL = onLine(analogRead(sensorPinLL));
  bool sensorCL = onLine(analogRead(sensorPinCL));
  bool sensorC = onLine(analogRead(sensorPinC));
  bool sensorCR = onLine(analogRead(sensorPinCR));
  bool sensorRR = onLine(analogRead(sensorPinRR));

  if (!sensorLL && !sensorCL && !sensorC && !sensorCR && !sensorRR)
  {
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    exit;
  }
  else if (sensorLL && !sensorCL && !sensorC && !sensorCR && !sensorRR)
  {
    return 900;
  }
  else if (sensorLL && sensorCL && !sensorC && !sensorCR && !sensorRR)
  {
    return 800;
  }
  else if (!sensorLL && sensorCL && !sensorC && !sensorCR && !sensorRR)
  {
    return 700;
  }
  else if (!sensorLL && sensorCL && sensorC && !sensorCR && !sensorRR)
  {
    return 600;
  }
  else if (!sensorLL && !sensorCL && sensorC && !sensorCR && !sensorRR)
  {
    return 500;
  }
  else if (!sensorLL && !sensorCL && sensorC && sensorCR && !sensorRR)
  {
    return 400;
  }
  else if (!sensorLL && !sensorCL && !sensorC && sensorCR && !sensorRR)
  {
    return 300;
  }
  else if (!sensorLL && !sensorCL && !sensorC && sensorCR && sensorRR)
  {
    return 200;
  }
  else if (!sensorLL && !sensorCL && !sensorC && !sensorCR && sensorRR)
  {
    return 100;
  }
}

bool onLine(float sensorRaw)
{
    Serial.println(sensorRaw);
    if (sensorRaw > calibrationValue) // Add calibration stuff here
    {
        return true;
    }
    else
    {
        return false;
    }
}
