#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int request = 1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

const int sensorPinL = A0;
const int sensorPinR = A1;

int sensorL, sensorR;

void setup()
{
    AFMS.begin();
    //
    // start the serial port
    //
    long baudRate = 9600;   // NOTE1: The baudRate for sending & receiving programs must match
    Serial.begin(baudRate); // NOTE2: Set the baudRate to 115200 for faster communication
    Serial.setTimeout(1);
    
    pinMode(sensorPinL, INPUT);
    pinMode(sensorPinR, INPUT);

    // Only continue while serial connection is available
    // This means nothing happens until the Python program is run
//    while (!Serial.available())
//    {
//    }

//    while (request == 0)
//    {
//        request = Serial.readString().toInt();
//    }

    motorL->setSpeed(50*request);
    motorR->setSpeed(50*request);
}


void loop()
{
//    if (Serial.readString().toInt() == 0)
//    {
//        motorL->run(RELEASE);
//        motorR->run(RELEASE);
//        exit;
//    }
    
    sensorL = onLine(analogRead(sensorPinL));
    sensorR = onLine(analogRead(sensorPinR));
    
    if (!sensorL && !sensorR)
    {
      motorL->run(FORWARD);
      motorR->run(FORWARD);
    }
    else if (!sensorL && sensorR)
    {
      motorL->run(FORWARD);
      motorR->run(RELEASE);
    }
    else if (sensorL && !sensorR)
    {
      motorL->run(RELEASE);
      motorR->run(FORWARD);
    }
    else if (sensorL && sensorR)
    {
        motorL->run(RELEASE);
        motorR->run(RELEASE);
    }

}

bool onLine(float sensorRaw)
{
    Serial.println(sensorRaw);
    if (sensorRaw > 500) // Add calibration stuff here
    {
        return true;
    }
    else
    {
        return false;
    }
}
